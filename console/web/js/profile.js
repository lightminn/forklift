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

// stateAgeS: since the browser last received a state; poseAgeS: the age the vehicle
// reports for the pose it sent (null when it does not report one).
export function staleness({ stateAgeS, poseAgeS }) {
  if (stateAgeS === null || stateAgeS === undefined) return { stale: true, text: "상태를 받지 못했습니다" };
  const age = Math.max(stateAgeS, poseAgeS ?? 0);
  if (age <= STALE_S) return { stale: false, text: "" };
  return { stale: true, text: `위치 ${age.toFixed(1)} s 전 · 지금 위치가 아닐 수 있습니다` };
}
