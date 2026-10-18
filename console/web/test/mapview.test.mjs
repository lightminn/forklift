import { test } from "node:test";
import assert from "node:assert/strict";
import { contentBounds, createMapView, fitView, panBy, screenToWorld, syncView, worldToScreen, zoomAt, zoomPercent } from "../js/mapview.js";

const near = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${a} != ${b}`);

test("fit centres the world and fills the tighter axis", () => {
  const v = createMapView();
  fitView(v, 14, 9, 1400, 600, 0);
  near(v.scale, 600 / 9);
  const [x0, y0] = worldToScreen(v, 1400, 600, 0, 0);
  const [x1, y1] = worldToScreen(v, 1400, 600, 14, 9);
  near((x0 + x1) / 2, 700);
  near(y0, 600); // world y=0 at the bottom edge
  near(y1, 0);
  near(zoomPercent(v), 100);
});

test("screenToWorld inverts worldToScreen", () => {
  const v = createMapView();
  fitView(v, 14, 9, 800, 500, 0.05);
  const [px, py] = worldToScreen(v, 800, 500, 3.2, 7.1);
  const [x, y] = screenToWorld(v, 800, 500, px, py);
  near(x, 3.2);
  near(y, 7.1);
});

test("zooming keeps the world point under the cursor fixed", () => {
  const v = createMapView();
  fitView(v, 14, 9, 800, 500, 0);
  const before = screenToWorld(v, 800, 500, 120, 340);
  zoomAt(v, 800, 500, 120, 340, 2.5);
  const after = screenToWorld(v, 800, 500, 120, 340);
  near(before[0], after[0], 1e-9);
  near(before[1], after[1], 1e-9);
  near(zoomPercent(v), 250);
});

test("zoom is clamped between 25 % and 800 % of the fitted scale", () => {
  const v = createMapView();
  fitView(v, 14, 9, 800, 500, 0);
  zoomAt(v, 800, 500, 400, 250, 1000);
  near(zoomPercent(v), 800);
  zoomAt(v, 800, 500, 400, 250, 1e-6);
  near(zoomPercent(v), 25);
});

test("panning moves the view with the pointer and stops following the truck", () => {
  const v = createMapView();
  fitView(v, 14, 9, 800, 500, 0);
  v.follow = true;
  const [wx, wy] = screenToWorld(v, 800, 500, 400, 250);
  panBy(v, 50, -20);
  const [px, py] = worldToScreen(v, 800, 500, wx, wy);
  near(px, 450);
  near(py, 230);
  assert.equal(v.follow, false);
});

test("an unfitted view reports no zoom and maps nothing", () => {
  const v = createMapView();
  assert.equal(zoomPercent(v), null);
  assert.equal(v.fitted, false);
});

test("fit honours a world that does not start at the origin (ROS map origin)", () => {
  const v = createMapView();
  fitView(v, 20, 10, 1000, 500, 0, -12.5, -3);
  near(v.cx, -2.5);
  near(v.cy, 2);
  const [x0, y0] = worldToScreen(v, 1000, 500, -12.5, -3);
  near(x0, 0);
  near(y0, 500);
});

test("the content bounds cover the grid window, the pallets, the destinations, the truck and the plan", () => {
  const grid = { head: { origin_m: [-2, -2], width: 80, height: 80, resolution_m: 0.05 } }; // a 4 m window
  const targets = { pallets: [{ pose: { x_m: 6, y_m: 1, yaw_rad: 0 } }], destinations: [{ pose: { x_m: -5, y_m: 7, yaw_rad: 0 } }] };
  const state = { pose: { x_m: 0, y_m: 0, yaw_rad: 0 }, plan: { poses: [[0, 0, 1], [3, -4, 1]] } };
  const b = contentBounds(grid, targets, state);
  assert.ok(b.x0 <= -6 && b.x0 + b.w >= 7 && b.y0 <= -4 && b.y0 + b.h >= 8, JSON.stringify(b));
  const empty = contentBounds(null, null, null);
  assert.deepEqual(empty, { x0: 0, y0: 0, w: 14, h: 9 });
});

test("the view fits the content until the operator moves it, then the content moving does not move it", () => {
  const v = createMapView();
  syncView(v, { x0: 0, y0: 0, w: 4, h: 4 }, 800, 500);
  near(v.cx, 2);
  syncView(v, { x0: 1, y0: 0, w: 4, h: 4 }, 800, 500); // the truck's grid window slid: still fitting
  near(v.cx, 3);
  panBy(v, 40, 0);
  const kept = { cx: v.cx, cy: v.cy, scale: v.scale };
  syncView(v, { x0: 2, y0: 1, w: 4, h: 4 }, 800, 500);
  assert.deepEqual({ cx: v.cx, cy: v.cy, scale: v.scale }, kept);
  zoomAt(v, 800, 500, 100, 100, 1.5);
  const zoomed = { cx: v.cx, cy: v.cy, scale: v.scale };
  syncView(v, { x0: 3, y0: 1, w: 4, h: 4 }, 800, 500);
  assert.deepEqual({ cx: v.cx, cy: v.cy, scale: v.scale }, zoomed);
});

test("fit view (fitted = false) fits the content again and resumes fitting", () => {
  const v = createMapView();
  syncView(v, { x0: 0, y0: 0, w: 4, h: 4 }, 800, 500);
  panBy(v, 300, 0);
  v.fitted = false; // the 맞춤 보기 button
  syncView(v, { x0: -10, y0: 0, w: 20, h: 8 }, 800, 500);
  near(v.cx, 0);
  near(v.cy, 4);
  near(zoomPercent(v), 100);
  syncView(v, { x0: -9, y0: 0, w: 20, h: 8 }, 800, 500);
  near(v.cx, 1);
});

test("a resized canvas keeps the zoom and the centre", () => {
  const v = createMapView();
  syncView(v, { x0: 0, y0: 0, w: 4, h: 4 }, 800, 500);
  zoomAt(v, 800, 500, 400, 250, 2);
  const { cx, cy } = v;
  syncView(v, { x0: 0, y0: 0, w: 4, h: 4 }, 600, 500);
  near(zoomPercent(v), 200);
  near(v.cx, cx);
  near(v.cy, cy);
});
