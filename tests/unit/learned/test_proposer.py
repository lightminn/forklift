"""The learned proposer's ONNX decoding (stage 3 plan, task 5) with a fake session."""
import numpy as np
import pytest

from forklift_learned import proposer as P

H, W = 480, 640
NM = 32


def _session(dets, *, mask_boxes=None):
    """A fake YOLO-seg session: dets = [(cx, cy, w, h, score)], each instance's mask is its box (via protos)."""
    n = len(dets)
    preds = np.zeros((1, 4 + 1 + NM, max(n, 1) + 3), np.float32)  # a few zero-score anchors
    protos = np.full((1, NM, H // 4, W // 4), -10.0, np.float32)
    for k, (cx, cy, w, h, s) in enumerate(dets):
        preds[0, :4, k] = (cx, cy, w, h)
        preds[0, 4, k] = s
        preds[0, 5 + k, k] = 1.0
        x0, y0, x1, y1 = (mask_boxes[k] if mask_boxes else (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2))
        protos[0, k, int(y0) // 4:int(y1) // 4, int(x0) // 4:int(x1) // 4] = 10.0

    class S:
        def get_inputs(self):
            class I:
                name = "images"
                shape = [1, 3, H, W]
            return [I()]

        def run(self, _names, feeds):
            assert feeds["images"].shape == (1, 3, H, W) and feeds["images"].dtype == np.float32
            return [preds, protos]

    return S()


class _Scene:
    rgb = np.zeros((H, W, 3), np.uint8)


def test_detections_become_learned_proposals_with_box_masks_and_scores():
    prop = P.LearnedProposer(session=_session([(200, 200, 100, 80, 0.9), (450, 300, 120, 60, 0.6)]))
    out = prop.propose(_Scene())
    assert [p.source for p in out] == ["learned", "learned"]
    assert [round(p.score, 2) for p in out] == [0.9, 0.6]
    m = out[0].mask
    assert m.shape == (H, W) and m[200, 200] and not m[200, 300] and abs(int(m.sum()) - 100 * 80) < 800


def test_low_scores_are_dropped_and_overlaps_suppressed():
    prop = P.LearnedProposer(session=_session([(200, 200, 100, 80, 0.9), (205, 200, 100, 80, 0.8),
                                               (450, 300, 120, 60, 0.01)]), conf=0.05, iou=0.6)
    out = prop.propose(_Scene())
    assert [round(p.score, 2) for p in out] == [0.9]


def test_the_masks_are_cropped_to_their_boxes():
    prop = P.LearnedProposer(session=_session([(200, 200, 100, 80, 0.9)], mask_boxes=[(100, 100, 400, 400)]))
    m = prop.propose(_Scene())[0].mask
    assert not m[120, 120] and m[200, 200]


def test_at_most_eight_proposals_highest_score_first():
    dets = [(40 + 70 * k, 240, 50, 50, 0.5 + 0.01 * k) for k in range(9)]
    out = P.LearnedProposer(session=_session(dets)).propose(_Scene())
    assert len(out) == 8 and out[0].score == pytest.approx(0.58)


def test_a_failure_falls_back_to_the_geometric_proposer():
    class Broken:
        def get_inputs(self):
            raise RuntimeError("no model")

    calls = []
    got = P.propose_with_fallback(_Scene(), P.LearnedProposer(session=Broken()),
                                  lambda scene: calls.append(scene) or "geometric")
    assert got.source == "geometric_fallback" and got.reason.startswith("RuntimeError") and calls


def test_the_box_crop_rounds_its_edges_like_ultralytics_predict():
    # ws1 equivalence on the trained yolo11n: Ultralytics' predict crop keeps rows/cols in [round(y1), round(y2)) --
    # a box from y 253.4 keeps row 253; cropping at the raw edge lost a row of thin far pallets (IoU 0.78-0.95)
    prop = P.LearnedProposer(session=_session([(300.0, 260.0, 120.0, 13.2, 0.9)], mask_boxes=[(200, 200, 400, 320)]))
    m = prop.propose(_Scene())[0].mask
    rows = np.nonzero(m.any(axis=1))[0]
    cols = np.nonzero(m.any(axis=0))[0]
    assert (rows.min(), rows.max()) == (253, 266)  # y 253.4 -> 253, y 266.6 -> 267 (exclusive)
    assert (cols.min(), cols.max()) == (240, 359)


class _FakeOrt:
    """A stand-in onnxruntime: records the preload and the providers asked for; ``active`` is what loads."""

    def __init__(self, active):
        self.active, self.preloaded, self.asked = active, False, None

    def preload_dlls(self, **kw):
        self.preloaded = True

    def get_available_providers(self):
        return ["TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"]

    class SessionOptions:
        pass

    def InferenceSession(self, path, sess_options=None, providers=None):
        self.asked = providers
        ort = self

        class S:
            def get_providers(self):
                return list(ort.active)

        return S()


def test_cuda_is_preloaded_and_pinned_to_the_device_asked(monkeypatch):
    # ws1: without torch imported first, onnxruntime-gpu could not find the pip CUDA 13 / cuDNN libraries and ran
    # on the CPU silently; the device is named explicitly (the AMD iGPU is never a CUDA device, but say which one)
    ort = _FakeOrt(["CUDAExecutionProvider", "CPUExecutionProvider"])
    monkeypatch.setitem(__import__("sys").modules, "onnxruntime", ort)
    prop = P.LearnedProposer("m.onnx", providers=["CUDAExecutionProvider", "CPUExecutionProvider"], device_id=1)
    assert ort.preloaded and ort.asked[0] == ("CUDAExecutionProvider", {"device_id": 1})
    assert prop.active_providers == ["CUDAExecutionProvider", "CPUExecutionProvider"]


def test_a_cuda_request_that_lands_on_the_cpu_is_an_error(monkeypatch):
    ort = _FakeOrt(["CPUExecutionProvider"])
    monkeypatch.setitem(__import__("sys").modules, "onnxruntime", ort)
    with pytest.raises(RuntimeError, match="CUDAExecutionProvider"):
        P.LearnedProposer("m.onnx", providers=["CUDAExecutionProvider", "CPUExecutionProvider"])


def test_the_default_providers_leave_out_tensorrt(monkeypatch):
    ort = _FakeOrt(["CPUExecutionProvider"])
    monkeypatch.setitem(__import__("sys").modules, "onnxruntime", ort)
    P.LearnedProposer("m.onnx")
    assert ort.asked == ["CPUExecutionProvider"] and not ort.preloaded
