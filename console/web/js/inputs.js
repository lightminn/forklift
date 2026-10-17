// Merges touch, keyboard and gamepad into one teleop input (design 5.2, 5.5).
// Pure module: main.js feeds it events and reads it once per animation frame.

const DEADZONE = 0.08;

// Raw Gamepad API object -> booleans; null for a missing or non-standard pad
// (axis indices differ on non-standard layouts, which could command full speed).
export function padSnapshot(gamepad) {
  if (!gamepad || gamepad.mapping !== "standard") return null;
  const pressed = (i) => {
    const b = gamepad.buttons?.[i];
    return Boolean(b && (b.pressed === true || (typeof b.value === "number" && b.value > 0.5)));
  };
  return {
    id: String(gamepad.id ?? "gamepad"),
    index: Number.isInteger(gamepad.index) ? gamepad.index : -1,
    axes: Array.from(gamepad.axes ?? [], (a) => (Number.isFinite(a) ? a : 0)),
    buttons: { deadman: pressed(5) || pressed(7), liftUp: pressed(12), liftDown: pressed(13), stop: pressed(1) },
  };
}

// Controls that were on screen and left it (tab switch, width change). The page's first
// frame has no "before": a page opened on another tab has not hidden anything.
export function hiddenEdge(wasShown, shown) {
  return wasShown === true && !shown;
}

export function createInputs({ onRelease }) {
  let touch = { deadman: false, x: 0, y: 0, lift: 0 };
  let keys = {};
  let pad = null;
  // After a release every source must report "not pressed" once before it counts again.
  const latched = { touch: false, keyboard: false, gamepad: false };

  function release(reason) {
    latched.touch = latched.keyboard = latched.gamepad = true;
    onRelease(reason);
  }

  function unlatch(source, pressed) {
    if (latched[source] && !pressed) latched[source] = false;
  }

  return {
    setTouch(partial) {
      touch = { ...touch, ...partial };
      unlatch("touch", touch.deadman);
    },
    setKeys(partial) {
      keys = { ...keys, ...partial };
      unlatch("keyboard", Boolean(keys.deadman));
    },
    setPad(snapshot) {
      if (snapshot === null) {
        if (pad !== null) {
          pad = null;
          release("gamepad_lost");
        }
        return;
      }
      if (pad !== null && (snapshot.index !== pad.index || snapshot.id !== pad.id)) {
        pad = snapshot; // a different pad took over: release as if the old one was lost
        release("gamepad_lost");
        return;
      }
      pad = snapshot;
      unlatch("gamepad", pad.buttons.deadman);
    },
    // gamepaddisconnected for the pad in use: release at once, even if the same index
    // is back before the next poll (the Gamepad API may reuse indices)
    padDisconnected(index) {
      if (pad !== null && pad.index === index) {
        pad = null;
        release("gamepad_lost");
      }
    },
    visibility(hidden) {
      if (hidden) release("hidden");
    },
    focus(focused) {
      if (!focused) release("blur");
    },
    // the teleop controls and their feedback left the screen (tab switch)
    controlsHidden() {
      release("controls_hidden");
    },
    read() {
      const ax = pad ? pad.axes : [];
      const steer = ax.length > 2 ? ax[2] : (ax[0] ?? 0);
      const padSpeed = -(ax[1] ?? 0);
      const padActive = pad !== null && (Math.abs(padSpeed) > DEADZONE || Math.abs(steer) > DEADZONE || pad.buttons.deadman);
      const keyActive = ["w", "a", "s", "d", "r", "f", "deadman"].some((k) => keys[k]);
      let source, deadman, speedRatio, curvatureRatio, liftDir;
      if (padActive) {
        source = "gamepad";
        deadman = pad.buttons.deadman && !latched.gamepad;
        speedRatio = Math.abs(padSpeed) > DEADZONE ? padSpeed : 0;
        curvatureRatio = Math.abs(steer) > DEADZONE ? -steer : 0;
        liftDir = pad.buttons.liftUp ? 1 : pad.buttons.liftDown ? -1 : 0;
      } else if (keyActive) {
        source = "keyboard";
        deadman = Boolean(keys.deadman) && !latched.keyboard;
        speedRatio = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
        curvatureRatio = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);
        liftDir = keys.r ? 1 : keys.f ? -1 : 0;
      } else {
        source = "touch";
        deadman = Boolean(touch.deadman) && !latched.touch;
        speedRatio = touch.y;
        curvatureRatio = -touch.x;
        liftDir = touch.lift;
      }
      if (!deadman) {
        speedRatio = 0;
        curvatureRatio = 0;
        liftDir = 0;
      }
      const clamp = (v) => Math.max(-1, Math.min(1, Number.isFinite(v) ? v : 0));
      return { deadman, speedRatio: clamp(speedRatio), curvatureRatio: clamp(curvatureRatio), liftDir, source, padId: pad?.id ?? null };
    },
  };
}
