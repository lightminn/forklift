import { test } from "node:test";
import assert from "node:assert/strict";
import { hiddenEdge, createInputs } from "../js/inputs.js";
import { createFrameTracker } from "../js/frames.js";
import { padSnapshot } from "../js/inputs.js";

test("visibility hidden releases and requests disarm", () => {
  const released = [];
  const i = createInputs({ onRelease: (r) => released.push(r) });
  i.setTouch({ deadman: true, x: 0, y: 0.8 });
  assert.equal(i.read().deadman, true);
  i.visibility(true);
  assert.deepEqual(released, ["hidden"]);
  const r = i.read();
  assert.equal(r.deadman, false);
  assert.equal(r.speedRatio, 0);
});

test("held touch stays released until pressed again", () => {
  const i = createInputs({ onRelease: () => {} });
  i.setTouch({ deadman: true, x: 0, y: 0.8 });
  i.focus(false);
  i.focus(true);
  i.setTouch({ deadman: true, x: 0, y: 0.8 });   // same press reported again
  assert.equal(i.read().deadman, false);
  i.setTouch({ deadman: false, x: 0, y: 0 });
  i.setTouch({ deadman: true, x: 0, y: 0.8 });   // a new press
  assert.equal(i.read().deadman, true);
});

test("gamepad disconnect releases", () => {
  const released = [];
  const i = createInputs({ onRelease: (r) => released.push(r) });
  i.setPad({ id: "pad", axes: [0, -1, 0.5, 0], buttons: { deadman: true, liftUp: false, liftDown: false, stop: false } });
  assert.equal(i.read().source, "gamepad");
  i.setPad(null);
  assert.deepEqual(released, ["gamepad_lost"]);
});

test("keyboard overrides idle touch; gamepad axes override keyboard", () => {
  const i = createInputs({ onRelease: () => {} });
  i.setKeys({ deadman: true, w: true, d: true });
  let r = i.read();
  assert.equal(r.speedRatio, 1); assert.equal(r.curvatureRatio, -1); assert.equal(r.source, "keyboard");
  i.setPad({ id: "pad", axes: [0, 0.5, 0, 0], buttons: { deadman: true, liftUp: false, liftDown: false, stop: false } });
  r = i.read();
  assert.equal(r.speedRatio, -0.5); assert.equal(r.source, "gamepad");
});

test("frame is displayed only after the next animation frame", () => {
  const queue = [];
  const f = createFrameTracker({ raf: (cb) => queue.push(cb) });
  f.reset(1);
  f.drawn(1, 5);
  assert.equal(f.displayedFrameId, null);
  queue.shift()();
  assert.equal(f.displayedFrameId, 5);
});

test("a new session drops the displayed frame and pending callbacks", () => {
  const queue = [];
  const f = createFrameTracker({ raf: (cb) => queue.push(cb) });
  f.reset(1);
  f.drawn(1, 5);
  f.reset(2);
  queue.shift()();
  assert.equal(f.displayedFrameId, null);
  f.drawn(1, 6);  // late frame from the old session
  queue.shift()?.();
  assert.equal(f.displayedFrameId, null);
});

test("a decode that finishes after a newer session is not drawn", () => {
  const queue = [];
  const f = createFrameTracker({ raf: (cb) => queue.push(cb) });
  f.reset(1);
  // decode of (1, 8) starts; meanwhile the vehicle restarts and (2, 1) is drawn
  f.reset(2);
  assert.equal(f.shouldDraw(2, 1), true);
  f.drawn(2, 1); queue.shift()();
  assert.equal(f.shouldDraw(1, 8), false);   // old session's decode completes late
  assert.equal(f.shouldDraw(2, 1), false);   // not newer than what is shown
  assert.equal(f.shouldDraw(2, 2), true);
});

test("gamepad buttons are read from .pressed, not object truthiness", () => {
  const off = { pressed: false, value: 0 }, on = { pressed: true, value: 1 };
  const raw = { id: "pad", connected: true, mapping: "standard", axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => off) };
  assert.equal(padSnapshot(raw).buttons.deadman, false);
  raw.buttons = raw.buttons.map((b, i) => (i === 5 ? on : b));
  assert.equal(padSnapshot(raw).buttons.deadman, true);
  assert.equal(padSnapshot(null), null);
  assert.equal(padSnapshot({ ...raw, mapping: "" }), null);  // non-standard layout is ignored
});

test("switching to another gamepad releases like a disconnect", () => {
  const released = [];
  const i = createInputs({ onRelease: (r) => released.push(r) });
  const btn = { deadman: true, liftUp: false, liftDown: false, stop: false };
  i.setPad({ id: "pad A", index: 0, axes: [0, -1, 0, 0], buttons: btn });
  assert.equal(i.read().deadman, true);
  i.setPad({ id: "pad B", index: 1, axes: [0, -1, 0, 0], buttons: btn });
  assert.deepEqual(released, ["gamepad_lost"]);
  assert.equal(i.read().deadman, false); // B's held button counts only after a release
});

test("padSnapshot keeps the gamepad index", () => {
  const off = { pressed: false, value: 0 };
  const raw = { id: "pad", index: 2, connected: true, mapping: "standard", axes: [0, 0, 0, 0], buttons: Array.from({ length: 17 }, () => off) };
  assert.equal(padSnapshot(raw).index, 2);
});

test("a disconnect event for the pad in use releases even if it reconnects before the next poll", () => {
  const released = [];
  const i = createInputs({ onRelease: (r) => released.push(r) });
  const held = { deadman: true, liftUp: false, liftDown: false, stop: false };
  i.setPad({ id: "pad A", index: 0, axes: [0, -1, 0, 0], buttons: held });
  i.padDisconnected(0);
  assert.deepEqual(released, ["gamepad_lost"]);
  i.setPad({ id: "pad A", index: 0, axes: [0, -1, 0, 0], buttons: held }); // same index again
  assert.equal(i.read().deadman, false);
});

test("a frame drawn while the camera is not on screen never counts as displayed", () => {
  const queue = [];
  let visible = false;
  const f = createFrameTracker({ raf: (cb) => queue.push(cb), visible: () => visible });
  f.reset(1);
  f.drawn(1, 5); // camera window collapsed, scrolled away or another view open
  queue.shift()?.();
  assert.equal(f.displayedFrameId, null);
  visible = true;
  f.drawn(1, 6);
  visible = false; // hidden before the next animation frame: still not seen
  queue.shift()?.();
  assert.equal(f.displayedFrameId, null);
  visible = true;
  f.drawn(1, 7);
  queue.shift()();
  assert.equal(f.displayedFrameId, 7);
});

test("hiding the controls releases every source until each is let go", () => {
  const released = [];
  const i = createInputs({ onRelease: (r) => released.push(r) });
  i.setKeys({ deadman: true, w: true });
  assert.equal(i.read().deadman, true);
  i.controlsHidden();
  assert.deepEqual(released, ["controls_hidden"]);
  assert.equal(i.read().deadman, false); // still held, but latched off
  i.setKeys({ deadman: false, w: false });
  i.read();
  i.setKeys({ deadman: true });
  assert.equal(i.read().deadman, true);
});

test("only controls that were on screen and left it release; a page opening elsewhere does not", () => {
  // the first frame of a page opened on another tab used to count as 'hidden', which
  // latched the keyboard until its first press was let go (C3 live run)
  assert.equal(hiddenEdge(null, false), false);
  assert.equal(hiddenEdge(null, true), false);
  assert.equal(hiddenEdge(true, false), true);
  assert.equal(hiddenEdge(false, false), false);
  assert.equal(hiddenEdge(true, true), false);
});
