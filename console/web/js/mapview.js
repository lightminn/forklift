// Map view transform: CSS pixels per metre around a world centre. Display only.

const MIN_ZOOM = 0.25;
const MAX_ZOOM = 8;

export function createMapView() {
  // auto: the view keeps fitting the content until the operator pans or zooms it (or follows the truck)
  return { scale: null, fitScale: null, cx: 0, cy: 0, follow: false, fitted: false, auto: true };
}

// World rectangle worth showing: the grid (Isaac sends only the window around the truck), the pallets and
// destinations, the truck and the plan. Nothing known yet: a 14 x 9 m floor.
export function contentBounds(grid, targets, state) {
  const xs = [];
  const ys = [];
  const add = (x, y, r = 0) => {
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    xs.push(x - r, x + r);
    ys.push(y - r, y + r);
  };
  if (grid?.head) {
    const [x0, y0] = grid.head.origin_m ?? [0, 0];
    add(x0, y0);
    add(x0 + grid.head.width * grid.head.resolution_m, y0 + grid.head.height * grid.head.resolution_m);
  }
  for (const p of [...(targets?.pallets ?? []), ...(targets?.destinations ?? [])]) if (p.pose) add(p.pose.x_m, p.pose.y_m, 1);
  if (state?.pose) add(state.pose.x_m, state.pose.y_m, 1);
  // B-P2c: while the start is unknown, the start's candidates (base poses) are what is worth showing
  for (const c of state?.slam?.reloc?.candidates ?? []) add(c.base[0], c.base[1], 1);
  for (const [x, y] of state?.plan?.poses ?? []) add(x, y);
  if (!xs.length) return { x0: 0, y0: 0, w: 14, h: 9 };
  const x0 = Math.min(...xs);
  const y0 = Math.min(...ys);
  return { x0, y0, w: Math.max(1, Math.max(...xs) - x0), h: Math.max(1, Math.max(...ys) - y0) };
}

// Called each draw: fit the content when not fitted yet or while the view is still fitting; a resized canvas
// keeps the zoom and the centre; a view the operator moved stays where it is.
export function syncView(v, b, w, h, margin = 0.04) {
  const key = `${b.x0},${b.y0},${b.w}x${b.h}`;
  const size = `${w}x${h}`;
  if (!v.fitted || (v.auto && v.contentKey !== key)) {
    fitView(v, b.w, b.h, w, h, margin, b.x0, b.y0);
    v.auto = true;
  } else if (v.sizeKey !== size) {
    const keep = v.scale / v.fitScale;
    v.fitScale = Math.min(w / b.w, h / b.h) * (1 - 2 * margin);
    v.scale = v.fitScale * keep;
  }
  v.contentKey = key;
  v.sizeKey = size;
  return v;
}

// (x0, y0) is the world's lower-left corner, e.g. a ROS map origin.
export function fitView(v, worldW, worldH, w, h, margin = 0.04, x0 = 0, y0 = 0) {
  v.fitScale = Math.min(w / worldW, h / worldH) * (1 - 2 * margin);
  v.scale = v.fitScale;
  v.cx = x0 + worldW / 2;
  v.cy = y0 + worldH / 2;
  v.fitted = true;
  return v;
}

export function worldToScreen(v, w, h, x, y) {
  return [w / 2 + (x - v.cx) * v.scale, h / 2 - (y - v.cy) * v.scale];
}

export function screenToWorld(v, w, h, px, py) {
  return [v.cx + (px - w / 2) / v.scale, v.cy - (py - h / 2) / v.scale];
}

// Zoom by `factor` keeping the world point under (px, py) where it is.
export function zoomAt(v, w, h, px, py, factor) {
  const [wx, wy] = screenToWorld(v, w, h, px, py);
  const lo = v.fitScale * MIN_ZOOM;
  const hi = v.fitScale * MAX_ZOOM;
  v.scale = Math.min(hi, Math.max(lo, v.scale * factor));
  v.cx = wx - (px - w / 2) / v.scale;
  v.cy = wy + (py - h / 2) / v.scale;
  v.auto = false;
  return v;
}

export function panBy(v, dx, dy) {
  v.cx -= dx / v.scale;
  v.cy += dy / v.scale;
  v.follow = false;
  v.auto = false;
  return v;
}

export function zoomPercent(v) {
  return v.fitted ? (v.scale / v.fitScale) * 100 : null;
}
