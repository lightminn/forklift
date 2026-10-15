// What this connection can do (welcome.capabilities, contract 3) and whether the
// picture on screen is still current. Display rules only; the vehicle decides.

// control: false is the monitor-only profile (gateway C2a): nothing on this page acts,
// STOP included.
export function controlProfile(capabilities) {
  if (!capabilities) return { control: false, mission: false, teleop: false, lift: false };
  const control = capabilities.control !== false;
  return {
    control,
    mission: control && capabilities.mission === true,
    teleop: control && (capabilities.teleop_direct === true || capabilities.teleop_video === true),
    lift: control && capabilities.lift === true,
  };
}

export const STALE_S = 1.0;

// stateAgeS: since the browser last received a state; poseAgeS: how old the pose already
// was when the vehicle sent that state (null when it does not say). The pose on screen is
// as old as both together.
export function staleness({ stateAgeS, poseAgeS }) {
  if (stateAgeS === null || stateAgeS === undefined) return { stale: true, text: "상태를 받지 못했습니다" };
  const age = stateAgeS + (poseAgeS ?? 0);
  if (age <= STALE_S) return { stale: false, text: "" };
  return { stale: true, text: `위치 ${age.toFixed(1)} s 전 · 지금 위치가 아닐 수 있습니다` };
}

// linkWelcome is the welcome of the control connection that is open now (null while
// reconnecting). Only that connection's own word turns STOP into the "cannot stop" notice;
// an old monitor-only welcome must never swallow a STOP meant for a vehicle that can stop.
export function stopIsRefused(linkWelcome) {
  return linkWelcome?.capabilities?.control === false;
}

// A pose or scan in another frame than the grid would be drawn misaligned (e.g. odom
// before map -> odom is known). Draw each only in the grid's frame; with no grid, draw
// the pose alone and a scan only in the pose's frame.
export function frameAgreement({ gridFrame, poseFrame, scanFrame }) {
  const base = gridFrame ?? poseFrame ?? null;
  const pose = !poseFrame || !gridFrame || poseFrame === gridFrame;
  const scan = Boolean(scanFrame) && (base === null || scanFrame === base);
  const note = gridFrame && poseFrame && poseFrame !== gridFrame ? `위치가 ${poseFrame} 좌표라 ${gridFrame} 지도 위에 그리지 않습니다` : null;
  return { pose, scan, note };
}

export function scanDrawable(scan) {
  return Boolean(scan) && scan.ageS <= STALE_S;
}
