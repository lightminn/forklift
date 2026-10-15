// Map view transform: CSS pixels per metre around a world centre. Display only.

const MIN_ZOOM = 0.25;
const MAX_ZOOM = 8;

export function createMapView() {
  return { scale: null, fitScale: null, cx: 0, cy: 0, follow: false, fitted: false };
}

export function fitView(v, worldW, worldH, w, h, margin = 0.04) {
  v.fitScale = Math.min(w / worldW, h / worldH) * (1 - 2 * margin);
  v.scale = v.fitScale;
  v.cx = worldW / 2;
  v.cy = worldH / 2;
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
  return v;
}

export function panBy(v, dx, dy) {
  v.cx -= dx / v.scale;
  v.cy += dy / v.scale;
  v.follow = false;
  return v;
}

export function zoomPercent(v) {
  return v.fitted ? (v.scale / v.fitScale) * 100 : null;
}
