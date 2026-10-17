// Tracks which camera frame is on screen (contract 4.2, design 5.3).
// A frame counts as displayed only after the next animation-frame callback following its
// draw, and only for the current session: a restarted vehicle reuses frame numbers.
// `visible` says whether the camera is on screen now (window open, in the viewport, tab
// shown); a frame drawn into a hidden canvas was not seen and never counts.

export function createFrameTracker({ raf, visible = () => true }) {
  let epoch = null;
  let generation = 0; // bumps on reset so callbacks scheduled before it are ignored
  let lastDrawn = null;
  let displayed = null;

  return {
    reset(sessionEpoch) {
      epoch = sessionEpoch;
      generation += 1;
      lastDrawn = null;
      displayed = null;
    },
    // Call after decoding, right before drawing: only the current session and only newer frames.
    shouldDraw(sessionEpoch, frameId) {
      return sessionEpoch === epoch && (lastDrawn === null || frameId > lastDrawn);
    },
    drawn(sessionEpoch, frameId) {
      if (sessionEpoch !== epoch) return;
      lastDrawn = frameId;
      if (!visible()) return;
      const gen = generation;
      raf(() => {
        if (gen === generation && sessionEpoch === epoch && visible()) displayed = frameId;
      });
    },
    get displayedFrameId() {
      return displayed;
    },
  };
}
