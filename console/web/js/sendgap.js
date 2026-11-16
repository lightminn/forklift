// The console's teleop send-gap meter (arrival-day rehearsal 2 R2-3 (b)). A teleop command is good for 0.30 s from the
// state tick it echoes, so from the console's own send the margin is about 0.2 s: a longer gap stops the truck (the
// MCU's DRIVE expiry) and voids a profile trial. The meter keeps the gaps between sends that ended in the last
// window and says when one passed the warning level; the page shows it and each teleop carries the page's report.

export function createSendGapMeter({ windowMs = 10000, warnMs = 200 } = {}) {
  let last = null;
  let gaps = []; // [endedAt, gapMs]

  function onSend(now) {
    const gap = last === null ? null : Math.round((now - last) * 10) / 10;
    if (gap !== null) gaps.push([now, gap]);
    last = now;
    gaps = gaps.filter(([t]) => now - t <= windowMs);
    return gap;
  }

  function stats(now) {
    const recent = gaps.filter(([t]) => now - t <= windowMs).map(([, g]) => g).sort((a, b) => a - b);
    if (recent.length === 0) return { count: 0, p99_ms: null, max_ms: null, warn: false };
    const p99 = recent[Math.min(recent.length - 1, Math.floor(0.99 * recent.length))];
    const max = recent[recent.length - 1];
    return { count: recent.length, p99_ms: p99, max_ms: max, warn: max > warnMs };
  }

  // the gap a send now would close (what the teleop built before the send reports), without recording it
  function pending(now) {
    return last === null ? null : Math.round((now - last) * 10) / 10;
  }

  function reset() {
    last = null;
    gaps = [];
  }

  return { onSend, stats, reset, pending };
}

// What a teleop tells the vehicle of the page that sent it (recorded only, never a decision input).
export function pageReport(vis, focus, gapMs) {
  return { vis: vis === "hidden" ? "hidden" : "visible", focus: Boolean(focus), gap_ms: gapMs ?? null };
}
