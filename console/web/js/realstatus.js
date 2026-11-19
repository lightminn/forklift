// The real truck's status message (docs/design/2026-11-19-real-safety-wiring.md 5, 5c-1): what the console reads
// from real_status besides the arbiter's state -- the control generation, the lease holder and the checks the
// operator bypassed for the session (shown as long as they are, "우회 중: ..."). Pure: no DOM.

export const CHECK_LABELS = {
  imu: "IMU",
  layer_timing: "층 시각",
  measurement: "계측",
  odometry: "오도메트리",
  slam: "SLAM",
  snapshot: "보정 세대",
  operator_start: "시작 확인",
  watch_current: "전류 감시",
  watch_wheel: "바퀴 감시",
  d5_pocket: "포켓 확인",
};

// The session's view after a real_status: {accepted, controlGen, leaseHeld}. A status of another session is not
// taken; a status without a holder (null) or naming another client means this console holds no lease.
export function applyRealStatus(msg, epoch, clientId) {
  if (!msg || msg.t !== "real_status" || msg.session_epoch !== epoch) return { accepted: false };
  const holder = msg.lease ? msg.lease.holder : null;
  return { accepted: true, controlGen: msg.control_gen, leaseHeld: holder === clientId };
}

// The banner text: empty when nothing is bypassed.
export function bypassBanner(status) {
  const list = status && Array.isArray(status.bypassed) ? status.bypassed : [];
  if (list.length === 0) return "";
  return `우회 중: ${list.map((c) => CHECK_LABELS[c] ?? c).join(", ")}`;
}
