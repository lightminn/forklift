// Top-down map: occupancy grid, LiDAR points, plan as a road with nodes, pallets and
// destinations as stations, the truck with its mode glow and tag, and the teleop
// predicted sweep. Display only; nothing here decides motion.

import { contentBounds, syncView, worldToScreen } from "./mapview.js";
import { palletKindLabel } from "./profile.js";

export function fitCanvas(cv) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const rc = cv.getBoundingClientRect();
  const w = Math.max(1, Math.round(rc.width * dpr));
  const h = Math.max(1, Math.round(rc.height * dpr));
  if (cv.width !== w || cv.height !== h) {
    cv.width = w;
    cv.height = h;
  }
  return { w, h, dpr };
}

// Predicted sweep of the body corners for the commanded curvature (rear-axle model).
export function predictSweep(pose, curvature, direction, chassis, steps = 30, ds = 0.08) {
  let { x_m: x, y_m: y, yaw_rad: yaw } = pose;
  const left = [];
  const right = [];
  const front = direction >= 0 ? chassis.nose_from_rear_m : -chassis.rear_overhang_m;
  const half = chassis.half_width_m;
  for (let i = 0; i <= steps; i += 1) {
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    left.push([x + c * front - s * half, y + s * front + c * half]);
    right.push([x + c * front + s * half, y + s * front - c * half]);
    x += direction * ds * c;
    y += direction * ds * s;
    yaw += direction * ds * curvature;
  }
  return { left, right };
}

// World rectangle covered by the grid: lower-left corner and size, metres.
export function worldBounds(grid) {
  if (!grid) return { x0: 0, y0: 0, w: 14, h: 9 };
  const [x0, y0] = grid.head.origin_m ?? [0, 0];
  return { x0, y0, w: grid.head.width * grid.head.resolution_m, h: grid.head.height * grid.head.resolution_m };
}

const MODE_TOK = { AUTO: "accent", TELEOP: "teleop", STOPPED: "crit" };

function roundRect(c, x, y, w, h, r) {
  c.beginPath();
  c.moveTo(x + r, y);
  c.arcTo(x + w, y, x + w, y + h, r);
  c.arcTo(x + w, y + h, x, y + h, r);
  c.arcTo(x, y + h, x, y, r);
  c.arcTo(x, y, x + w, y, r);
  c.closePath();
}

function tag(c, text, x, y, color, tok) {
  c.font = `600 11px ${tok.font}`;
  const tw = c.measureText(text).width;
  const w = tw + 24;
  roundRect(c, x - w / 2, y, w, 19, 6);
  c.fillStyle = "rgba(13,15,20,0.9)";
  c.fill();
  c.strokeStyle = color;
  c.lineWidth = 1;
  c.stroke();
  c.fillStyle = color;
  c.beginPath();
  c.arc(x - w / 2 + 9, y + 9.5, 3, 0, Math.PI * 2);
  c.fill();
  c.fillStyle = tok.fg;
  c.textAlign = "left";
  c.textBaseline = "middle";
  c.fillText(text, x - w / 2 + 16, y + 10);
  c.textBaseline = "alphabetic";
}

export function drawMap(cv, model, tok, view, layers) {
  const { w: W, h: H, dpr } = fitCanvas(cv);
  const w = W / dpr;
  const h = H / dpr;
  const c = cv.getContext("2d");
  const grid = model.grid;
  const B = worldBounds(grid); // the grid's own rectangle (its cells and lines)
  syncView(view, contentBounds(grid, model.targets, model.state), w, h);
  const st = model.state;
  if (view.follow && st?.pose) {
    view.cx = st.pose.x_m;
    view.cy = st.pose.y_m;
  }
  const P = (x, y) => worldToScreen(view, w, h, x, y);
  const sc = view.scale;
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.fillStyle = tok["map-bg"];
  c.fillRect(0, 0, w, h);

  // grid cells: free floor slightly lifted, occupied in blue, unknown left dark
  if (grid && layers.grid) {
    const { width: nx, height: ny, resolution_m: res } = grid.head;
    const values = grid.head.values ?? { unknown: 0, free: 1, occupied: 2 };
    const cells = grid.cells;
    const cell = res * sc + 0.6;
    for (const [value, color] of [[values.free, tok["map-free"]], [values.occupied, tok.obst]]) {
      c.fillStyle = color;
      for (let j = 0; j < ny; j += 1) {
        for (let i = 0; i < nx; i += 1) {
          if (cells[j * nx + i] !== value) continue;
          const [x, y] = P(B.x0 + i * res, B.y0 + (j + 1) * res);
          if (x > w || y > h || x + cell < 0 || y + cell < 0) continue;
          c.fillRect(x, y, cell, cell);
        }
      }
    }
  }
  // metre lines, brighter every 5 m
  for (const [step, color, lw] of [[1, tok["map-grid"], 1], [5, tok["line-2"], 1]]) {
    c.strokeStyle = color;
    c.lineWidth = lw;
    c.beginPath();
    for (let x = Math.ceil(B.x0 / step) * step; x <= B.x0 + B.w + 1e-9; x += step) {
      const [a, b] = P(x, B.y0);
      const [, d] = P(x, B.y0 + B.h);
      c.moveTo(Math.round(a) + 0.5, b);
      c.lineTo(Math.round(a) + 0.5, d);
    }
    for (let y = Math.ceil(B.y0 / step) * step; y <= B.y0 + B.h + 1e-9; y += step) {
      const [a, b] = P(B.x0, y);
      const [cc] = P(B.x0 + B.w, y);
      c.moveTo(a, Math.round(b) + 0.5);
      c.lineTo(cc, Math.round(b) + 0.5);
    }
    c.stroke();
  }

  const targets = model.targets;
  const mission = st?.mission;
  const label = (text, x, y, color) => {
    c.font = `500 11px ${tok.font}`;
    c.fillStyle = color;
    c.textAlign = "center";
    c.fillText(text, x, y);
  };
  if (targets) {
    // destinations: dashed bays with an approach stub
    for (const d of targets.destinations) {
      const sel = mission && mission.destination_id === d.destination_id && ["running", "paused", "held", "resuming"].includes(mission.status);
      const [x, y] = P(d.pose.x_m, d.pose.y_m);
      const s = 0.9 * sc;
      c.save();
      c.translate(x, y);
      c.rotate(-d.pose.yaw_rad);
      c.fillStyle = tok["station-fill"];
      c.strokeStyle = sel ? tok.accent : tok.dest;
      c.lineWidth = sel ? 2 : 1.4;
      c.setLineDash([5, 4]);
      c.fillRect(-s / 2, -s / 2, s, s);
      c.strokeRect(-s / 2, -s / 2, s, s);
      c.setLineDash([]);
      c.restore();
      label(`하역 ${d.destination_id}`, x, y + s / 2 + 15, sel ? tok.accent : tok.dest);
    }
    // pallets: station boxes with slats and an insertion stub on the approach side
    for (const p of targets.pallets) {
      if (p.state === "carried" || !p.pose) continue;
      const sel = mission && mission.pallet_id === p.pallet_id && ["running", "paused", "held", "resuming"].includes(mission.status);
      const [x, y] = P(p.pose.x_m, p.pose.y_m);
      const sized = Number.isFinite(p.depth_m) && Number.isFinite(p.width_m);
      const dw = (sized ? p.depth_m : 0.6) * sc; // unknown size: a nominal box drawn dashed
      const ws = (sized ? p.width_m : 0.6) * sc;
      c.save();
      c.translate(x, y);
      c.rotate(-p.pose.yaw_rad);
      c.strokeStyle = tok.dest;
      c.lineWidth = 2;
      c.beginPath();
      c.moveTo(-dw / 2, 0);
      c.lineTo(-dw / 2 - 0.45 * sc, 0);
      c.stroke();
      c.fillStyle = tok.node;
      c.beginPath();
      c.arc(-dw / 2 - 0.45 * sc, 0, 3.5, 0, Math.PI * 2);
      c.fill();
      c.fillStyle = tok["station-fill"];
      c.strokeStyle = sel ? tok.accent : p.state === "delivered" ? tok.ok : tok.station;
      c.lineWidth = sel ? 2.2 : 1.4;
      if (sel) {
        c.shadowColor = tok.accent;
        c.shadowBlur = 12;
      }
      c.fillRect(-dw / 2, -ws / 2, dw, ws);
      c.shadowBlur = 0;
      if (!sized) c.setLineDash([4, 3]);
      c.strokeRect(-dw / 2, -ws / 2, dw, ws);
      c.setLineDash([]);
      c.strokeStyle = tok.muted;
      c.globalAlpha = 0.5;
      c.lineWidth = 1;
      c.beginPath();
      for (const f of [-0.3, 0, 0.3]) {
        c.moveTo(-dw / 2 + 3, f * ws);
        c.lineTo(dw / 2 - 3, f * ws);
      }
      c.stroke();
      c.globalAlpha = 1;
      c.restore();
      label(`${p.pallet_id} · ${palletKindLabel(p.kind)}`, x, y + Math.max(dw, ws) / 2 + 15, sel ? tok.accent : tok.station);
    }
  }

  // plan as a road: wide grey band, darker core, reverse stretches dashed amber, nodes at cusps
  const plan = st?.plan;
  if (layers.plan && plan && plan.poses.length > 1 && st.mode !== "TELEOP") {
    const pts = plan.poses.map(([x, y, dir]) => [...P(x, y), dir]);
    c.lineCap = "round";
    c.lineJoin = "round";
    c.strokeStyle = tok.road;
    c.lineWidth = Math.max(7, 0.22 * sc);
    c.beginPath();
    pts.forEach(([x, y], i) => (i ? c.lineTo(x, y) : c.moveTo(x, y)));
    c.stroke();
    c.strokeStyle = tok["road-in"];
    c.lineWidth = Math.max(3, 0.1 * sc);
    c.stroke();
    c.strokeStyle = tok.warn;
    c.lineWidth = 2;
    c.setLineDash([6, 5]);
    c.beginPath();
    for (let i = 1; i < pts.length; i += 1) {
      if (pts[i][2] >= 0) continue;
      c.moveTo(pts[i - 1][0], pts[i - 1][1]);
      c.lineTo(pts[i][0], pts[i][1]);
    }
    c.stroke();
    c.setLineDash([]);
    c.lineCap = "butt";
    const nodes = [pts[0], pts[pts.length - 1]];
    for (let i = 2; i < pts.length; i += 1) if (Math.sign(pts[i][2]) !== Math.sign(pts[i - 1][2])) nodes.push(pts[i - 1]);
    for (const [x, y] of nodes) {
      c.fillStyle = tok["road-in"];
      c.strokeStyle = tok.node;
      c.lineWidth = 2;
      c.beginPath();
      c.arc(x, y, 5, 0, Math.PI * 2);
      c.fill();
      c.stroke();
    }
  }

  if (layers.sweep && st?.pose && st.mode === "TELEOP" && model.chassis) {
    const cmd = st.command || { speed_mps: 0, curvature_inv_m: 0 };
    const dir = cmd.speed_mps < 0 ? -1 : 1;
    const sweep = predictSweep(st.pose, cmd.curvature_inv_m, dir, model.chassis);
    c.strokeStyle = tok.teleop;
    c.lineWidth = 2;
    c.setLineDash([4, 4]);
    for (const side of [sweep.left, sweep.right]) {
      c.beginPath();
      side.forEach(([x, y], i) => {
        const [px, py] = P(x, y);
        if (i) c.lineTo(px, py);
        else c.moveTo(px, py);
      });
      c.stroke();
    }
    c.setLineDash([]);
  }

  if (layers.lidar) {
    c.fillStyle = tok.lidar;
    c.globalAlpha = model.scanApprox ? 0.4 : 1; // placed with a later pose: may be off
    const pts = model.scanPoints || [];
    for (let i = 0; i < pts.length; i += 2) {
      const [x, y] = P(pts[i], pts[i + 1]);
      c.fillRect(x - 1.2, y - 1.2, 2.4, 2.4);
    }
    c.globalAlpha = 1;
  }

  // truck: glow in the mode colour, chevron for heading, forks ahead, tag below
  if (st?.pose && model.chassis) {
    const ch = model.chassis;
    const p = st.pose;
    const color = model.stale ? tok.muted : tok[MODE_TOK[st.mode] ?? "robot"];
    if (model.stale) c.globalAlpha = 0.45; // an old pose is not a stopped truck
    const [x, y] = P(p.x_m, p.y_m);
    const bodyFront = ch.nose_from_rear_m - 0.36;
    const hw = ch.half_width_m * sc;
    c.save();
    c.translate(x, y);
    c.rotate(-p.yaw_rad);
    c.fillStyle = tok.station;
    for (const fy of [-0.17, 0.17]) c.fillRect(bodyFront * sc, (fy - 0.0275) * sc, 0.36 * sc, Math.max(2, 0.055 * sc));
    if (st.load?.state === "loaded" && targets) {
      const lp = targets.pallets.find((q) => q.pallet_id === st.load.pallet_id);
      if (lp && Number.isFinite(lp.depth_m) && Number.isFinite(lp.width_m)) {
        const off = ch.nose_from_rear_m - 0.33; // back face of the load: fork tip minus insertion depth
        c.fillStyle = "rgba(245,165,36,0.18)";
        c.strokeStyle = tok.dest;
        c.lineWidth = 1.6;
        c.fillRect(off * sc, (-lp.width_m / 2) * sc, lp.depth_m * sc, lp.width_m * sc);
        c.strokeRect(off * sc, (-lp.width_m / 2) * sc, lp.depth_m * sc, lp.width_m * sc);
      }
    }
    roundRect(c, -ch.rear_overhang_m * sc, -hw, (ch.rear_overhang_m + bodyFront) * sc, 2 * hw, Math.min(10, 0.12 * sc));
    c.shadowColor = color;
    c.shadowBlur = 22;
    c.fillStyle = tok.panel;
    c.fill();
    c.shadowBlur = 0;
    c.strokeStyle = color;
    c.lineWidth = 2;
    c.stroke();
    const mid = ((bodyFront - ch.rear_overhang_m) / 2) * sc;
    const a = Math.min(hw * 0.7, 0.2 * sc);
    c.strokeStyle = tok.fg;
    c.lineWidth = Math.max(2, 0.05 * sc);
    c.lineCap = "round";
    c.lineJoin = "round";
    c.beginPath();
    c.moveTo(mid - a * 0.5, -a);
    c.lineTo(mid + a * 0.5, 0);
    c.lineTo(mid - a * 0.5, a);
    c.stroke();
    c.lineCap = "butt";
    c.restore();
    const reach = (ch.nose_from_rear_m + 0.1) * sc;
    tag(c, `${model.vehicleId ?? "차량"} · ${model.stale ? "위치 지연" : st.mode ?? "관찰"}`, x, y + Math.max(reach * 0.55, hw) + 8, color, tok);
    c.globalAlpha = 1;
  }

  // scale bar, top-left
  const meters = sc > 120 ? 0.5 : sc > 40 ? 1 : sc > 12 ? 5 : 10;
  c.fillStyle = tok.fg;
  c.fillRect(14, 16, meters * sc, 2);
  c.fillRect(14, 12, 1.5, 8);
  c.fillRect(14 + meters * sc - 1.5, 12, 1.5, 8);
  c.font = `500 10.5px ${tok.font}`;
  c.textAlign = "left";
  c.fillText(`${meters} m`, 18 + meters * sc, 20);
}

// Pallet under a screen point, if any (for picking on the map).
export function palletAt(view, w, h, targets, px, py) {
  if (!targets || !view.fitted) return null;
  for (const p of targets.pallets) {
    if (!p.pose) continue;
    const [x, y] = worldToScreen(view, w, h, p.pose.x_m, p.pose.y_m);
    const dx = px - x;
    const dy = py - y;
    const c = Math.cos(p.pose.yaw_rad);
    const s = Math.sin(p.pose.yaw_rad);
    // screen y is flipped: rotate into the pallet frame
    const lx = (dx * c - dy * s) / view.scale;
    const ly = (-dx * s - dy * c) / view.scale;
    const d = Number.isFinite(p.depth_m) ? p.depth_m : 0.6;
    const wd = Number.isFinite(p.width_m) ? p.width_m : 0.6;
    if (Math.abs(lx) <= d / 2 + 0.1 && Math.abs(ly) <= wd / 2 + 0.1) return p;
  }
  return null;
}
