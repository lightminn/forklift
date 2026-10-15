import { test } from "node:test";
import assert from "node:assert/strict";
import { createMapView, fitView, panBy, screenToWorld, worldToScreen, zoomAt, zoomPercent } from "../js/mapview.js";

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
