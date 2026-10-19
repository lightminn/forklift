// Task assignment: the templates the vehicle supports and the rules for each wizard
// step. The vehicle still checks every request (contract 5); these only shape the page.

export const PHASES = [
  ["observe", "관찰"],
  ["approach", "접근"],
  ["insert", "삽입"],
  ["lift", "들기"],
  ["extract", "빼기"],
  ["transport", "운반"],
  ["lower", "내리기"],
  ["withdraw", "후퇴"],
  ["return", "복귀"],
];

// Subtask index for each phase; 4 = after the task (return).
const SUBTASK_OF = { observe: 0, approach: 0, insert: 1, lift: 1, extract: 1, transport: 2, lower: 3, withdraw: 3, return: 4 };

// console/1 has one mission kind: pick a pallet and set it down at a destination.
const TEMPLATES = [
  {
    code: "PALLET_TRANSPORT",
    name: "팔레트 운반",
    needs: "mission",
    summary: "팔레트를 찾아 들고 하역 위치에 내려놓은 뒤 복귀합니다",
    subtasks: [
      { kind: "move", name: "팔레트 앞으로 이동 · 관찰" },
      { kind: "act", name: "포켓 삽입 · 들기" },
      { kind: "move", name: "하역 위치로 운반" },
      { kind: "act", name: "내리기 · 후퇴" },
    ],
  },
];

const ACTIVE = ["accepted", "preparing", "running", "paused", "held", "resuming", "recovering"];

// A pallet the operator may name in a submit (the vehicle checks again): on the floor, delivered or not -- or the one
// the state's load names on the forks, delivered as it is carried (persistent console plan D5 6c item 7).
export function selectable(pallet, load = null) {
  if (!pallet) return false;
  return pallet.state === "available" || pallet.state === "delivered" || carriedLabel(pallet, load) !== null;
}

export function carriedLabel(pallet, load) {
  const onForks = pallet?.state === "carried" && load?.state === "loaded" && load.pallet_id === pallet.pallet_id;
  return onForks ? "실린 팔레트 — 바로 운반" : null;
}

export function templatesFor(capabilities, query = "") {
  if (!capabilities) return [];
  const q = query.trim().toLowerCase();
  return TEMPLATES.filter((t) => capabilities[t.needs] === true).filter(
    (t) => !q || [t.name, t.code, t.summary, ...t.subtasks.map((s) => s.name)].some((s) => s.toLowerCase().includes(q)),
  );
}

export function nextAllowed(step, sel) {
  if (step === 1) return Boolean(sel.template);
  if (step === 2) return Boolean(sel.template && sel.pallet && sel.destination);
  return false;
}

// Why a task cannot be assigned now, or null. ctx: { state?, canMission?, hasLease, mode, mission }.
export function blockReason(ctx) {
  if ("state" in ctx && !ctx.state) return "차량에 연결되지 않았습니다";
  if (ctx.canMission === false) return "이 연결은 작업 지시를 받지 않습니다 (관찰 전용 또는 미지원)";
  if (!ctx.hasLease) return "조종 권한이 있어야 지시할 수 있습니다";
  if (ctx.mode === "STOPPED") return "정지 상태입니다 · 재무장 뒤 지시할 수 있습니다";
  if (ctx.mode === "TELEOP") return "수동 조종 중에는 지시할 수 없습니다";
  if (ctx.mission && ACTIVE.includes(ctx.mission.status)) return "작업이 진행 중입니다";
  if (ctx.state?.stopped_confirmed === false) return "정지 확인을 기다리는 중입니다"; // else not_stopped
  return null;
}

export function subtaskOf(phase) {
  return SUBTASK_OF[phase] ?? -1;
}

export function currentDestination(mi) {
  if (!mi || !ACTIVE.includes(mi.status)) return null;
  const s = subtaskOf(mi.phase);
  if (s === 0 || s === 1) return `팔레트 ${mi.pallet_id}`;
  if (s === 2 || s === 3) return `하역 위치 ${mi.destination_id}`;
  if (s === 4) return "복귀 위치";
  return null;
}

export function missionProgress(mi) {
  if (!mi) return 0;
  if (mi.status === "done") return 1;
  const idx = PHASES.findIndex(([en]) => en === mi.phase);
  return idx < 0 ? 0 : idx / PHASES.length;
}
