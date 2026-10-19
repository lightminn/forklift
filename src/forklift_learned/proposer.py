"""YOLO instance segmentation (ONNX) -> InstanceProposal (stage 3 plan, task 5; design §6).

The model is exported at the camera's grid (480 x 640, no letterbox), so the input is the RGB / 255 in CHW and
the masks come back on the same grid. Decoding follows the Ultralytics segment head: per anchor 4 box values
(cx, cy, w, h), one class score (``pallet``) and 32 mask coefficients; masks = sigmoid(coefficients @ prototypes)
upsampled x4, cropped to the box, thresholded at 0.5. Scores >= ``conf``, class-agnostic NMS at ``iou``, at most
``max_det`` (design §8). The proposal score is the detection confidence; identity is decided downstream against
tau (instance_fit._identity), never here. A failure falls back to the geometric proposer (design §10).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from forklift_core.perception.instance_proposals import InstanceProposal


def _iou(box, boxes):
    x0 = np.maximum(box[0], boxes[:, 0])
    y0 = np.maximum(box[1], boxes[:, 1])
    x1 = np.minimum(box[2], boxes[:, 2])
    y1 = np.minimum(box[3], boxes[:, 3])
    inter = np.clip(x1 - x0, 0, None) * np.clip(y1 - y0, 0, None)
    area = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])
    return inter / np.maximum(area(box) + area(boxes) - inter, 1e-9)


def nms(boxes, scores, iou):
    order = np.argsort(-scores, kind="stable")
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        order = order[1:][_iou(boxes[i], boxes[order[1:]]) <= iou]
    return keep


def _linear_weights(n_out: int, n_in: int) -> np.ndarray:
    """(n_out, n_in) bilinear weights with half-pixel centres (align_corners=False, as torch interpolate)."""
    x = (np.arange(n_out) + 0.5) * n_in / n_out - 0.5
    x = np.clip(x, 0, n_in - 1)
    lo = np.floor(x).astype(int)
    hi = np.minimum(lo + 1, n_in - 1)
    frac = x - lo
    w = np.zeros((n_out, n_in))
    w[np.arange(n_out), lo] += 1 - frac
    w[np.arange(n_out), hi] += frac
    return w


class LearnedProposer:
    def __init__(self, onnx_path=None, *, providers=None, conf: float = 0.05, iou: float = 0.6, max_det: int = 8,
                 session=None, threads: int | None = None):
        if session is None:
            import onnxruntime as ort  # noqa: PLC0415 -- the [learned] extra

            options = ort.SessionOptions()
            if threads:  # onnxruntime sizes its pool to the machine, not the Slurm allocation
                options.intra_op_num_threads, options.inter_op_num_threads = int(threads), 1
            session = ort.InferenceSession(str(onnx_path), sess_options=options,
                                           providers=providers or ort.get_available_providers())
        self.session, self.conf, self.iou, self.max_det = session, float(conf), float(iou), int(max_det)

    def propose(self, scene) -> list:
        inp = self.session.get_inputs()[0]
        rgb = np.asarray(scene.rgb, dtype=np.float32) / 255.0
        h, w = rgb.shape[:2]
        if list(inp.shape[2:]) != [h, w]:
            raise ValueError(f"model input {inp.shape} does not match the image {h}x{w}")
        preds, protos = self.session.run(None, {inp.name: rgb.transpose(2, 0, 1)[None]})[:2]
        p = preds[0].T  # (anchors, 4 + 1 + nm)
        scores = p[:, 4]
        sel = scores >= self.conf
        p, scores = p[sel], scores[sel]
        if not len(p):
            return []
        cx, cy, bw, bh = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
        boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1)
        keep = nms(boxes, scores, self.iou)[: self.max_det]
        proto = protos[0]
        nm, ph, pw = proto.shape
        wy, wx = _linear_weights(h, ph), _linear_weights(w, pw)
        rows, cols = np.arange(h)[:, None], np.arange(w)[None, :]
        out = []
        for rank, i in enumerate(keep):
            # Ultralytics' native-resolution masks: logits upsampled bilinearly, cropped to the box at pixel
            # indices (x0 <= c < x1), positive logit (sigmoid > 0.5)
            logits = wy @ (p[i, 5:5 + nm] @ proto.reshape(nm, -1)).reshape(ph, pw) @ wx.T
            x0, y0, x1, y1 = boxes[i]
            mask = (logits > 0.0) & (cols >= x0) & (cols < x1) & (rows >= y0) & (rows < y1)
            if mask.any():
                out.append(InstanceProposal(f"l{rank}", mask, float(np.clip(scores[i], 0.0, 1.0)), "learned"))
        return out


@dataclass(frozen=True)
class Proposed:
    proposals: list
    source: str          # "learned" or "geometric_fallback"
    reason: str | None = None
    extra: object = field(default=None)


def propose_with_fallback(scene, learned, geometric) -> Proposed:
    """The learned proposals, or the geometric fallback's on any learned failure (design §10)."""
    try:
        return Proposed(learned.propose(scene), "learned")
    except Exception as exc:  # noqa: BLE001 -- any model failure falls back
        return Proposed([], "geometric_fallback", f"{type(exc).__name__}: {exc}", geometric(scene))


__all__ = ["LearnedProposer", "Proposed", "nms", "propose_with_fallback"]
