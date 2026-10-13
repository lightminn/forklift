// Top-down map: occupancy grid, LiDAR points, plan, targets, truck footprint and the
// teleop predicted sweep. Display only; nothing here decides motion.

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

export function drawMap(cv, model, tok) {
  const { w, h, dpr } = fitCanvas(cv);
  const c = cv.getContext("2d");
  const grid = model.grid;
  const W = grid ? grid.head.width * grid.head.resolution_m : 14;
  const H = grid ? grid.head.height * grid.head.resolution_m : 9;
  const sc = Math.min(w / W, h / H);
  const ox = (w - W * sc) / 2;
  const oy = (h - H * sc) / 2;
  const X = (x) => ox + x * sc;
  const Y = (y) => oy + (H - y) * sc;
  c.setTransform(1, 0, 0, 1, 0, 0);
  c.fillStyle = tok["map-bg"];
  c.fillRect(0, 0, w, h);
  if (grid) {
    const { width: nx, height: ny, resolution_m: res } = grid.head;
    const cells = grid.cells;
    c.fillStyle = tok["map-obst"];
    for (let j = 0; j < ny; j += 1) {
      for (let i = 0; i < nx; i += 1) {
        const v = cells[j * nx + i];
        if (v === 2) c.fillRect(X(i * res), Y((j + 1) * res), res * sc + 0.5, res * sc + 0.5);
      }
    }
    c.fillStyle = tok["map-grid"];
    for (let j = 0; j < ny; j += 1) {
      for (let i = 0; i < nx; i += 1) {
        if (cells[j * nx + i] === 0) c.fillRect(X(i * res), Y((j + 1) * res), res * sc, res * sc);
      }
    }
  }
  c.strokeStyle = tok["map-grid"];
  c.lineWidth = 1;
  c.beginPath();
  for (let x = 0; x <= W; x += 1) {
    c.moveTo(X(x), Y(0));
    c.lineTo(X(x), Y(H));
  }
  for (let y = 0; y <= H; y += 1) {
    c.moveTo(X(0), Y(y));
    c.lineTo(X(W), Y(y));
  }
  c.stroke();
  c.strokeStyle = tok["map-obst"];
  c.lineWidth = 3 * dpr;
  c.strokeRect(X(0), Y(H), W * sc, H * sc);

  const font = (px, weight = 500) => `${weight} ${px * dpr}px ${tok.font}`;
  const targets = model.targets;
  if (targets) {
    for (const d of targets.destinations) {
      c.strokeStyle = tok.dest;
      c.lineWidth = 3 * dpr;
      c.beginPath();
      c.arc(X(d.pose.x_m), Y(d.pose.y_m), 0.55 * sc, 0, Math.PI * 2);
      c.stroke();
      c.fillStyle = tok.dest;
      c.font = font(11, 600);
      c.textAlign = "center";
      c.fillText(d.destination_id, X(d.pose.x_m), Y(d.pose.y_m) - 0.65 * sc);
    }
    const missionPallet = model.state?.mission?.pallet_id;
    for (const p of targets.pallets) {
      if (p.state === "carried") continue;
      const sel = p.pallet_id === missionPallet;
      c.save();
      c.translate(X(p.pose.x_m), Y(p.pose.y_m));
      c.rotate(-p.pose.yaw_rad);
      c.fillStyle = tok.panel;
      c.strokeStyle = sel ? tok.accent : tok.muted;
      c.lineWidth = (sel ? 2.5 : 1.5) * dpr;
      c.fillRect((-p.depth_m / 2) * sc, (-p.width_m / 2) * sc, p.depth_m * sc, p.width_m * sc);
      c.strokeRect((-p.depth_m / 2) * sc, (-p.width_m / 2) * sc, p.depth_m * sc, p.width_m * sc);
      c.restore();
      c.fillStyle = sel ? tok.accent : tok.muted;
      c.textAlign = "center";
      c.font = font(11, 600);
      c.fillText(p.pallet_id, X(p.pose.x_m), Y(p.pose.y_m) + (p.width_m / 2 + 0.3) * sc);
    }
  }

  const plan = model.state?.plan;
  if (plan && plan.poses.length > 1 && model.state.mode !== "TELEOP") {
    for (let i = 1; i < plan.poses.length; i += 1) {
      const [x0, y0] = plan.poses[i - 1];
      const [x1, y1, dir] = plan.poses[i];
      c.strokeStyle = dir < 0 ? tok.warn : tok.accent;
      c.lineWidth = 2.5 * dpr;
      c.setLineDash(dir < 0 ? [6 * dpr, 4 * dpr] : []);
      c.beginPath();
      c.moveTo(X(x0), Y(y0));
      c.lineTo(X(x1), Y(y1));
      c.stroke();
    }
    c.setLineDash([]);
  }

  const st = model.state;
  if (st && st.mode === "TELEOP" && model.chassis) {
    const cmd = st.command || { speed_mps: 0, curvature_inv_m: 0 };
    const dir = cmd.speed_mps < 0 ? -1 : 1;
    const sweep = predictSweep(st.pose, cmd.curvature_inv_m, dir, model.chassis);
    c.strokeStyle = tok.teleop;
    c.lineWidth = 2 * dpr;
    c.setLineDash([3 * dpr, 3 * dpr]);
    for (const side of [sweep.left, sweep.right]) {
      c.beginPath();
      side.forEach(([x, y], i) => (i ? c.lineTo(X(x), Y(y)) : c.moveTo(X(x), Y(y))));
      c.stroke();
    }
    c.setLineDash([]);
  }

  c.fillStyle = tok.lidar;
  const rr = 1.6 * dpr;
  const pts = model.scanPoints || [];
  for (let i = 0; i < pts.length; i += 2) c.fillRect(X(pts[i]) - rr / 2, Y(pts[i + 1]) - rr / 2, rr, rr);

  if (st && model.chassis) {
    const ch = model.chassis;
    const p = st.pose;
    c.save();
    c.translate(X(p.x_m), Y(p.y_m));
    c.rotate(-p.yaw_rad);
    c.fillStyle = tok.robot;
    c.strokeStyle = tok["robot-ink"];
    c.lineWidth = 1.2 * dpr;
    const bodyFront = ch.nose_from_rear_m - 0.36;
    c.beginPath();
    c.rect(-ch.rear_overhang_m * sc, -ch.half_width_m * sc, (ch.rear_overhang_m + bodyFront) * sc, 2 * ch.half_width_m * sc);
    c.fill();
    c.stroke();
    c.fillStyle = tok["robot-ink"];
    for (const fy of [-0.17, 0.17]) c.fillRect(bodyFront * sc, (fy - 0.0275) * sc, 0.36 * sc, 0.055 * sc);
    c.beginPath();
    c.lineWidth = 1.5 * dpr;
    c.moveTo(-0.2 * sc, 0);
    c.lineTo(0.25 * sc, 0);
    c.lineTo(0.13 * sc, -0.09 * sc);
    c.moveTo(0.25 * sc, 0);
    c.lineTo(0.13 * sc, 0.09 * sc);
    c.stroke();
    c.restore();
    if (st.load?.state === "loaded" && targets) {
      const lp = targets.pallets.find((q) => q.pallet_id === st.load.pallet_id);
      if (lp) {
        const off = ch.nose_from_rear_m - 0.33; // back face of the load: fork tip minus insertion depth
        c.save();
        c.translate(X(p.x_m + off * Math.cos(p.yaw_rad)), Y(p.y_m + off * Math.sin(p.yaw_rad)));
        c.rotate(-p.yaw_rad);
        c.strokeStyle = tok.accent;
        c.lineWidth = 2 * dpr;
        c.strokeRect(0, (-lp.width_m / 2) * sc, lp.depth_m * sc, lp.width_m * sc);
        c.restore();
      }
    }
  }
  c.fillStyle = tok.fg;
  c.fillRect(X(0.4), Y(0.35), sc, 2 * dpr);
  c.font = font(10);
  c.textAlign = "left";
  c.fillText("1 m", X(0.4), Y(0.35) - 5 * dpr);
}
