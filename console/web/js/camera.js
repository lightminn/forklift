// Camera view: draws the decoded frame, then display-only overlays computed from the
// vehicle's camera_model (contract 3). Overlays never feed a safety decision.

import { fitCanvas, predictSweep } from "./map.js";
import { cameraHeightM } from "./profile.js";

function projector(model, pose, height, w, h) {
  const cam = model.base_from_camera;
  const [fwd] = cam.xyz_m;
  const [, qy, , qw] = cam.quaternion_xyzw;
  const pitch = 2 * Math.atan2(qy, qw);
  const c = Math.cos(pose.yaw_rad);
  const s = Math.sin(pose.yaw_rad);
  const ox = pose.x_m + c * fwd;
  const oy = pose.y_m + s * fwd;
  const scale = w / model.width;
  const f = model.fx * scale;
  const u0 = w / 2;
  const v0 = (model.cy / model.height) * h;
  return (x, y, z) => {
    const dx = x - ox;
    const dy = y - oy;
    const dz = z - height;
    const fw = dx * c + dy * s;
    const lf = -dx * s + dy * c;
    const d = fw * Math.cos(pitch) - dz * Math.sin(pitch);
    const up = fw * Math.sin(pitch) + dz * Math.cos(pitch);
    if (d <= 0.12) return null;
    return [u0 + (f * -lf) / d, v0 - (f * up) / d];
  };
}

export function drawCamera(cv, view, tok) {
  const { w, h, dpr } = fitCanvas(cv);
  const c = cv.getContext("2d");
  c.setTransform(1, 0, 0, 1, 0, 0);
  c.fillStyle = "#0b0f12";
  c.fillRect(0, 0, w, h);
  if (view.bitmap) c.drawImage(view.bitmap, 0, 0, w, h);
  const st = view.state;
  const model = view.cameraModel;
  const height = model ? cameraHeightM(model, st) : null; // a carriage camera rides the lift
  if (st?.pose && model && height !== null && st.mode === "TELEOP" && view.chassis) {
    const cmd = st.command || { speed_mps: 0, curvature_inv_m: 0 };
    const dir = cmd.speed_mps < 0 ? -1 : 1;
    if (dir > 0) {
      const project = projector(model, st.pose, height, w, h);
      const sweep = predictSweep(st.pose, cmd.curvature_inv_m, 1, view.chassis, 34, 0.08);
      c.strokeStyle = "#b79cff";
      c.lineWidth = 3 * dpr;
      c.setLineDash([8 * dpr, 6 * dpr]);
      for (const side of [sweep.left, sweep.right]) {
        c.beginPath();
        let started = false;
        for (const [x, y] of side) {
          const q = project(x, y, 0.003);
          if (!q) {
            started = false;
            continue;
          }
          if (started) c.lineTo(q[0], q[1]);
          else c.moveTo(q[0], q[1]);
          started = true;
        }
        c.stroke();
      }
      c.setLineDash([]);
    }
  }
  c.font = `500 ${11 * dpr}px ${tok.font}`;
  const hud = (txt, x, y, align) => {
    c.textAlign = align;
    const tw = c.measureText(txt).width;
    c.fillStyle = "rgba(0,0,0,0.55)";
    c.fillRect(align === "left" ? x - 4 * dpr : x - tw - 4 * dpr, y - 12 * dpr, tw + 8 * dpr, 17 * dpr);
    c.fillStyle = "#e8edf0";
    c.fillText(txt, x, y);
  };
  if (view.frameId !== null && view.frameId !== undefined) {
    hud(`frame ${view.frameId}`, w - 10 * dpr, h - 10 * dpr, "right");
  }
}
