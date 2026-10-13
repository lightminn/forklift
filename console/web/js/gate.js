// Small pure gates used by main.js (design 5.3, contract 2).

// Direct (line-of-sight) teleop right after a video-staleness disarm needs an explicit
// "the truck is directly visible" confirmation; toggling modes never clears it.
export function createTeleopGate() {
  let mode = "direct";
  let armed = false;
  let videoUsed = false; // video mode was selected at some point since the last arm
  let needConfirm = false;
  return {
    setMode(next) {
      mode = next;
      if (next === "video" && armed) videoUsed = true;
    },
    // The page cannot tell a video expiry from other disarms (the vehicle judges video
    // age), so a disarm after video was used in that arm period asks for the
    // confirmation, whatever mode is selected when the disarm arrives.
    onArmed(nowArmed) {
      if (nowArmed && !armed) videoUsed = mode === "video";
      if (armed && !nowArmed && (videoUsed || mode === "video")) needConfirm = true;
      armed = nowArmed;
    },
    confirmDirect() {
      needConfirm = false;
    },
    canSend() {
      return !(mode === "direct" && needConfirm);
    },
    get mode() {
      return mode;
    },
    get needConfirm() {
      return needConfirm;
    },
  };
}

// One id per tab, kept across reloads. sessionStorage is copied into windows opened from
// this one, window.name is not: an id whose window.name marker is missing is a copy.
export function chooseClientId({ stored, windowName, newId }) {
  const mark = (id) => `forklift-console:${id}`;
  if (stored && windowName === mark(stored)) return { id: stored, windowName };
  const id = newId();
  return { id, windowName: mark(id) };
}
