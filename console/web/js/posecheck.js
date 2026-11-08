// The unconfirmed position check on the screen (docs/plans/2026-11-02-unconfirmed-runner.md 4, design
// docs/plans/2026-11-02-unconfirmed-position.md 1.4): the band while the start is not confirmed, the straight reverse's
// stroke left, and the hypothesis zones -- where a pallet would be relative to the truck if a look-alike place were
// the true one, drawn around the truck's estimate (not the map's pallets).
import { reasonText } from "./results.js";

const MAX_ZONES = 64; // drawn at most; the union can hold every hypothesis's pallet zone

const LOCKED = "확인 전 위치 · 전진·포크 잠김 · 뒤쪽 낮은 물체는 보이지 않습니다 · 차 뒤를 직접 확인하세요";

export function poseCheckOf(st) {
  const pc = st?.pose_check;
  // the shadow gates nothing (plan 2026-11-06 1.1): no band, no lock on the screen
  if (!pc || typeof pc.status !== "string" || pc.status === "confirmed" || pc.status.startsWith("shadow:")) return null;
  return pc;
}

function metres(v) {
  return Number.isFinite(v) ? `${Math.max(0, v).toFixed(2)} m` : "모름";
}

function strokeText(stroke) {
  if (!stroke) return "";
  if (stroke.state === "spent") return "확인 전 후진 거리를 다 썼습니다";
  if (stroke.state === "closed") return "한 번 누름의 거리를 다 썼습니다 · 차가 선 뒤 손을 뗐다가 다시 누르세요";
  return `곧은 후진 ${metres(stroke.left_m)} 남음 · 전체 ${metres(stroke.total_left_m)}`;
}

// {text, title, tone} or null: what the band says
export function poseBand(pc) {
  if (!pc) return null;
  if (pc.status === "checking") {
    const text = ["출발 위치를 확인하는 중입니다 · 움직이지 마세요", legend(pc)].filter(Boolean).join(" · ");
    return { text, title: "확인 전 위치", tone: "warn" };
  }
  if (pc.status.startsWith("ended:")) {
    const reason = pc.status.slice("ended:".length);
    return { text: reasonText(reason).text, title: reason, tone: "bad" };
  }
  const parts = [LOCKED];
  const stroke = strokeText(pc.stroke);
  if (stroke) parts.push(stroke);
  if (legend(pc)) parts.push(legend(pc));
  return { text: parts.join(" · "), title: "확인 전 위치", tone: "warn" };
}

// the legend of the map's dashed zones, in the band so it is always in view (Codex 2c P3), "" without zones
function legend(pc) {
  if (!poseZones(pc).length) return "";
  const omitted = Number.isFinite(pc.zones_omitted) && pc.zones_omitted > 0 ? ` · 먼 ${pc.zones_omitted} 개 생략` : "";
  return `점선은 가설 구역(차 기준)${omitted}`;
}

// The vehicle-side F1 block (docs/plans/2026-11-08-f1-vehicle-block.md 2.4): the state's start_block, latched for the
// whole session -- the map's pose looks aligned once a placement is confirmed, but nothing confirmed it.
export const START_BLOCK_TEXT = "자동 운행과 미션은 받지 않습니다 · 수동 운전만 됩니다";

export function startBand(st) {
  const b = st?.start_block;
  if (!b) return null;
  const head = b.source === "operator_placed" ? "운전자가 놓은 위치 · 확인되지 않음 · 지도 표시를 믿고 운전하지 마세요"
    : "출발 위치의 출처를 확인할 수 없습니다 · 지도 표시를 믿고 운전하지 마세요";
  return { text: `${head} · ${START_BLOCK_TEXT}`, title: b.reason, tone: "bad" };
}

// the zones to draw: polygons of [x, y] map points, finite, at most MAX_ZONES
export function poseZones(pc) {
  const out = [];
  for (const z of pc?.zones ?? []) {
    if (out.length >= MAX_ZONES) break;
    if (Array.isArray(z) && z.length >= 3 && z.every((p) => Array.isArray(p) && p.length === 2
        && Number.isFinite(p[0]) && Number.isFinite(p[1]))) out.push(z);
  }
  return out;
}
