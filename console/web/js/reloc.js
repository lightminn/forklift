// B-P2c (docs/plans/2026-10-29-start-relocalization.md 1.3): the start relocalization on the console. The vehicle
// says it in the state's slam.reloc while the start is unknown (the pose is null then): its state, the candidate
// set's generation, the attempts, the last reason, the time left, and the candidates an operator may pick (base
// poses in the map). Pure: the screen and the tests share it.
import { screenToWorld, worldToScreen } from "./mapview.js";

const STATE_KO = {
  relocalizing: "출발 위치 찾는 중",
  awaiting_pick: "출발 위치 후보",
  reloc_failed: "출발 위치를 정할 수 없음",
  stopped: "정지됨 · 다시 시도할 수 있음",
  localizing: "추정 위치로 SLAM 시작 중",
  // F1 (docs/plans/2026-10-31-operator-placed-start.md 1.2): the operator's placed pose
  placing: "놓은 자리 근처를 맞추는 중",
  place_preview: "놓은 자리 미리보기 · 스캔 겹침을 보고 확정하세요",
};
// why a placement's local re-match was refused (plan 1.3)
const PLACE_REASON_KO = {
  ambiguous: "이 자리는 스캔만으로 위치를 정할 수 없습니다 · 차를 옮긴 뒤 다시 시도하세요",
  edge: "놓은 자리에서 조금 떨어진 곳이 맞습니다 · 더 정확히 놓아 주세요",
  low_fit: "놓은 자리 근처에서 지도와 맞는 곳이 없습니다",
  refine_moved: "맞춘 위치가 흔들립니다 · 다시 놓아 주세요",
  empty: "놓은 자리 근처에 차가 설 수 있는 곳이 없습니다",
  uninformative: "스캔에 보이는 것이 너무 적습니다",
  timeout: "맞추는 시간이 지났습니다 · 다시 놓아 주세요",
  overflow: "맞추기가 너무 오래 걸립니다 · 다시 놓아 주세요",
};
const PLACE_DRAG_PX = 12; // a shorter drag sets no heading
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
  if (reloc.place_reason && ["awaiting_pick", "reloc_failed"].includes(reloc.state)) {
    parts.push(PLACE_REASON_KO[reloc.place_reason] ?? `놓기 거부 ${reloc.place_reason}`);
  }
  if (reloc.places && reloc.max_places) parts.push(`놓기 ${reloc.places}/${reloc.max_places}`);
  const left = clock(reloc.operator_left_s);
  if (left !== null && reloc.state !== "localizing") parts.push(`남은 시간 ${left}`);
  const title = [`세대 ${reloc.generation ?? "—"}`, `시도 ${reloc.attempts ?? 0}`,
                 reloc.last_reason ? `마지막 사유 ${reloc.last_reason}` : null].filter(Boolean).join(" · ");
  return { text: parts.join(" · "), title };
}

export function relocActions(reloc, hasLease) {
  // F1: a preview may be dropped for a new global attempt too (plan 2026-10-31-operator-placed-start 1.2)
  const waiting = ["awaiting_pick", "reloc_failed", "stopped", "place_preview"].includes(reloc?.state);
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

// One button per candidate in the banner: the map's click reaches only one of two candidates on the same spot with
// other headings (Codex B-P2c impl review P2); the heading in degrees tells them apart.
export function candidateButtons(reloc, actions) {
  if (reloc?.state !== "awaiting_pick") return [];
  return (reloc.candidates ?? []).map((c) => {
    const deg = Math.round(((c.base[2] * 180) / Math.PI + 360) % 360);
    return { index: c.index, label: `후보 ${c.index + 1} · ${Number(c.score).toFixed(2)} · ${deg}°`,
             enabled: Boolean(actions?.pick) };
  });
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

// F1: the operator's placed pose -- pressed where the truck stands, dragged along its heading (map frame).
export function placeFromDrag(view, w, h, down, up) {
  if (!view?.fitted || Math.hypot(up.x - down.x, up.y - down.y) < PLACE_DRAG_PX) return null;
  const [x, y] = screenToWorld(view, w, h, down.x, down.y);
  return [x, y, Math.atan2(-(up.y - down.y), up.x - down.x)]; // the screen's y grows down, the map's up
}

export function placeActions(reloc, hasLease) {
  const stage = ["awaiting_pick", "reloc_failed", "place_preview"].includes(reloc?.state);
  const left = (reloc?.places ?? 0) < (reloc?.max_places ?? 0);
  let why = "";
  if (!hasLease) why = "조종 권한이 있어야 합니다";
  else if (!stage) why = "후보를 기다리거나 미리보기를 볼 때만 놓을 수 있습니다";
  else if (!left) why = "놓기를 세 번 다 썼습니다 · 다시 시도하거나 세션을 끝내세요";
  const confirm = Boolean(hasLease) && reloc?.state === "place_preview" && reloc.preview_id != null;
  return { place: { enabled: Boolean(hasLease) && stage && left, why }, confirm };
}

// The two-step confirm's keys: the pose (rounded as sent) or the preview and the set and session they belong to.
export function placeKey(reloc, pose, base) {
  return { kind: "relocalize_place", epoch: base.epoch, leaseId: base.leaseId, controlGen: base.controlGen,
           generation: reloc.generation, pose: pose.map((v) => Math.round(v * 1000) / 1000) };
}

export function previewKey(reloc, base) {
  return { kind: "relocalize_place_confirm", epoch: base.epoch, leaseId: base.leaseId, controlGen: base.controlGen,
           generation: reloc.generation, preview_id: reloc.preview_id };
}
