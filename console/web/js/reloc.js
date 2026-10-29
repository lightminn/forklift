// B-P2c (docs/plans/2026-10-29-start-relocalization.md 1.3): the start relocalization on the console. The vehicle
// says it in the state's slam.reloc while the start is unknown (the pose is null then): its state, the candidate
// set's generation, the attempts, the last reason, the time left, and the candidates an operator may pick (base
// poses in the map). Pure: the screen and the tests share it.
import { worldToScreen } from "./mapview.js";

const STATE_KO = {
  relocalizing: "출발 위치 찾는 중",
  awaiting_pick: "출발 위치 후보",
  reloc_failed: "출발 위치를 정할 수 없음",
  stopped: "정지됨 · 다시 시도할 수 있음",
  localizing: "추정 위치로 SLAM 시작 중",
};
const PICK_RADIUS_M = 0.35;

export function relocOf(st) {
  return st?.slam?.reloc ?? null;
}

function clock(s) {
  if (!Number.isFinite(s)) return null;
  const t = Math.max(0, Math.ceil(s));
  return `${Math.floor(t / 60)}:${String(t % 60).padStart(2, "0")}`;
}

export function relocSummary(reloc) {
  if (!reloc) return { text: "", title: "" };
  const label = STATE_KO[reloc.state] ?? reloc.state;
  const parts = [label];
  if (reloc.state === "awaiting_pick") parts.push(`후보 ${reloc.candidates?.length ?? 0}개 · 지도에서 골라 주세요`);
  if (reloc.state === "relocalizing" && reloc.attempts) parts.push(`시도 ${reloc.attempts + 1}`);
  const left = clock(reloc.operator_left_s);
  if (left !== null && reloc.state !== "localizing") parts.push(`남은 시간 ${left}`);
  const title = [`세대 ${reloc.generation ?? "—"}`, `시도 ${reloc.attempts ?? 0}`,
                 reloc.last_reason ? `마지막 사유 ${reloc.last_reason}` : null].filter(Boolean).join(" · ");
  return { text: parts.join(" · "), title };
}

export function relocActions(reloc, hasLease) {
  const waiting = ["awaiting_pick", "reloc_failed", "stopped"].includes(reloc?.state);
  const pick = Boolean(hasLease) && reloc?.state === "awaiting_pick" && (reloc.candidates?.length ?? 0) > 0;
  let why = "";
  if (!waiting) why = "후보를 기다리거나 위치 찾기가 멈춘 때만 다시 시도할 수 있습니다";
  else if (!hasLease) why = "조종 권한이 있어야 합니다";
  return { pick, retry: { enabled: waiting && Boolean(hasLease), why } };
}

// The two-step confirm's key (plan D13 pattern): the candidate and the generation it was shown in, so a second click
// on a newer set arms again instead of sending -- the pick sent is the first click's (Codex B-P2c design P2).
export function pickKey(reloc, index, base) {
  return { kind: "relocalize_pick", epoch: base.epoch, leaseId: base.leaseId, controlGen: base.controlGen,
           generation: reloc.generation, candidate: index };
}

export function candidateAt(view, w, h, reloc, px, py) {
  if (!reloc || !view.fitted) return null;
  let best = null;
  let bestD = Infinity;
  for (const c of reloc.candidates ?? []) {
    const [x, y] = worldToScreen(view, w, h, c.base[0], c.base[1]);
    const d = Math.hypot(px - x, py - y) / view.scale;
    if (d <= PICK_RADIUS_M && d < bestD) {
      best = c.index;
      bestD = d;
    }
  }
  return best;
}
