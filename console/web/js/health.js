// Device status and alarms from what the console already receives (docs/plans/2026-10-22-console-status-panel.md).
// Pure module: no DOM, no clocks -- times come in as arguments. The limits are the console's own (profile.js,
// queue.js); the vehicle is not asked for anything new.

import { STALE_S, permitText, staleness, stopKind, videoStaleAfterS } from "./profile.js";
import { CALM_REASONS, SILENCE_MS } from "./queue.js";
import { slamLabel, standbyText } from "./results.js";

const OPEN_MS = 500; // warn/crit this long opens an alarm (plan D3; damping, not a limit)
const CLEAR_MS = 1000; // ok this long closes it
const GAP_MS = 2000; // calls further apart than this: what happened between was not seen
const KEEP = 100;
const RANK = { warn: 1, crit: 2 };
const LABELS = {
  control: "제어 연결", state: "상태 수신", pose: "위치", camera: "영상(최근 프레임 수신)", scan: "스캔", grid: "지도",
  media: "미디어 연결", feedback: "차량 피드백", stop: "정지", permit: "구동 허가", scene: "장면", lease: "조종 권한",
  slam: "SLAM 시작 검사", freshness: "차량 신선도", session: "세션", queue: "대기열",
};

const r = (key, level, value, detail = "") => ({ key, label: LABELS[key], level, value, detail });
const secs = (ms) => `${(ms / 1000).toFixed(1)} s 전`;

// The video limit's pacing factor applied to another limit (plan D2: a slow simulation sends scans further apart).
function paced(limitS, st) {
  return limitS * videoStaleAfterS(st); // videoStaleAfterS = 1 / factor, or 1 at real time
}

// A media row: declared or not, received since this welcome or not, and how old.
function mediaRow(key, declared, at, receivedAt, limitS, inp) {
  if (!declared) return r(key, "off", "차량이 주지 않음");
  const since = receivedAt !== null && receivedAt !== undefined && inp.welcomeAt !== null && receivedAt >= inp.welcomeAt;
  if (!since) {
    if (inp.welcomeAt === null) return r(key, "unknown", "연결 기다리는 중");
    const wait = Math.max(SILENCE_MS, limitS * 1000);
    return inp.nowMs - inp.welcomeAt <= wait ? r(key, "unknown", "첫 수신 기다리는 중") : r(key, "warn", "오지 않음");
  }
  const age = inp.nowMs - at;
  return r(key, age <= limitS * 1000 ? "ok" : "warn", secs(age), `한계 ${limitS.toFixed(1)} s`);
}

// The supervisor's standby (contract 6) says a start or a run failed: that wins over a reset or a start in progress.
function standbyFailed(s) {
  if (!s || s.standby !== true) return false;
  const last = s.last_exit;
  return Boolean((last && last.exit_code !== 0 && last.exit_code !== 3)
    || (s.phase === "stopped" && s.end !== "ended" && s.end !== "terminated"));
}

// Why the links are not expected now, or null: an operator's session end, a reset or a standby starting the next
// session -- unless the standby reports a failure (Codex plan round 2).
function notExpected(inp) {
  if (inp.link.up) return null;
  if (inp.sessionEnding === "session_end") return "세션 종료됨";
  if (standbyFailed(inp.standby)) return null;
  if (inp.sessionEnding === "sim_reset") return "장면 초기화 중";
  if (inp.standby?.standby === true) return "세션 기동 중";
  return null;
}

function sessionRow(inp) {
  const s = inp.standby;
  if (standbyFailed(s)) return r("session", "warn", standbyText(s) ?? "세션 실패");
  if (inp.sessionEnding === "session_end") return r("session", "off", "세션 종료됨");
  if (inp.sessionEnding === "sim_reset") return r("session", "off", "장면 초기화 중");
  if (s && s.standby === true) return r("session", "off", standbyText(s) ?? "세션 기동 중");
  const notes = Array.isArray(inp.state?.notes) && inp.state.notes.length ? inp.state.notes.join(" · ") : "";
  return r("session", "ok", notes || "이상 없음");
}

function queueRow(q) {
  if (!q) return r("queue", "off", "—");
  if (q.run === "running") return r("queue", "ok", "실행 중");
  if (q.items > 0 && !CALM_REASONS.includes(q.stopReason)) return r("queue", "warn", `대기열 멈춤: ${q.stopReason}`);
  return r("queue", "off", q.items > 0 ? `멈춤 · ${q.stopReason}` : "비어 있음");
}

function leaseValue(inp) {
  const st = inp.state;
  if (inp.hasLease) return "이 기기";
  if (!st || st.lease === null || st.lease === undefined) return "모름";
  if (st.lease.holder === null || st.lease.holder === undefined) return "아무도 쥐지 않음";
  return "다른 기기가 쥠";
}

function freshnessValue(st) {
  const f = st?.freshness;
  const parts = f ? Object.entries(f).filter(([, v]) => Number.isFinite(v)).map(([k, v]) => `${k} ${v.toFixed(1)} s`) : [];
  return parts.length ? parts.join(" · ") : "차량이 주지 않음";
}

// input: {nowMs, welcomeAt, link: {up, mediaUp}, sessionEnding, caps, monitor, hasLease, state, stateAt, cameraAt,
//         scanAt, scanReceivedAt, grid: {present, withdrawnReason}, standby, queue: {run, stopReason, items}}
export function healthRows(inp) {
  const st = inp.state;
  const media = inp.caps?.media ?? {};
  const rows = [];
  const pause = notExpected(inp);
  // a state counts only once this connection has sent one: an older one is kept by the session across a reconnect
  const fresh = Boolean(st) && inp.stateAt !== null && inp.welcomeAt !== null && inp.stateAt >= inp.welcomeAt
    && inp.link.up;
  if (pause) {
    for (const key of ["control", "media", "state", "pose", "camera", "scan"]) rows.push(r(key, "off", pause));
    return rows.concat(stateRows(inp, media, fresh, pause === "세션 종료됨"));
  }

  // the links
  if (!inp.link.up) rows.push(r("control", "crit", "끊김 · 재접속 중"));
  else if (inp.welcomeAt === null) rows.push(r("control", "warn", "연결 중"));
  else rows.push(r("control", "ok", "연결됨"));
  rows.push(r("media", inp.link.mediaUp ? "ok" : "warn", inp.link.mediaUp ? "연결됨" : "재접속 중"));

  // what arrives since this welcome
  if (!inp.link.up || inp.welcomeAt === null) rows.push(r("state", "unknown", "연결 기다리는 중"), r("pose", "unknown", "—"));
  else if (!fresh) {
    rows.push(inp.nowMs - inp.welcomeAt <= SILENCE_MS
      ? r("state", "unknown", "첫 상태 기다리는 중") : r("state", "crit", "welcome 뒤 상태가 오지 않음"));
    rows.push(r("pose", "unknown", "—"));
  } else {
    const age = inp.nowMs - inp.stateAt;
    rows.push(r("state", age <= STALE_S * 1000 ? "ok" : age <= SILENCE_MS ? "warn" : "crit", secs(age)));
    if (!st.pose) rows.push(r("pose", "warn", "위치 없음"));
    else {
      const s = staleness({ stateAgeS: age / 1000, poseAgeS: st.pose.age_s ?? null });
      rows.push(r("pose", s.stale ? "warn" : "ok", `${(age / 1000 + (st.pose.age_s ?? 0)).toFixed(1)} s 전`, st.pose.frame ?? ""));
    }
  }
  rows.push(mediaRow("camera", Boolean(media.camera_rgb), inp.cameraAt, inp.cameraAt, videoStaleAfterS(st), inp));
  rows.push(mediaRow("scan", Boolean(media.scan), inp.scanAt, inp.scanReceivedAt, paced(STALE_S, st), inp));
  return rows.concat(stateRows(inp, media, fresh, false));
}

// the grid and what the last state says -- that state only if this connection sent it (Codex plan round 3): before
// one, unknown; after the operator ended the session, off
function stateRows(inp, media, fresh, ended) {
  const st = fresh ? inp.state : null;
  const rows = [];
  if (!media.grid) rows.push(r("grid", "off", "차량이 주지 않음"));
  else if (inp.grid?.present) rows.push(r("grid", "ok", "있음"));
  else if (inp.grid?.withdrawnReason) rows.push(r("grid", "warn", `지도 철회됨(${inp.grid.withdrawnReason})`));
  else rows.push(r("grid", "unknown", "아직 없음"));

  // what the state says
  if (!fresh) {
    for (const key of ["feedback", "stop", "permit", "scene"]) {
      rows.push(ended ? r(key, "off", "세션 종료됨") : r(key, "unknown", "이 연결의 상태를 아직 받지 못함"));
    }
    return rows.concat(shownRows(inp));
  }
  const fb = st.feedback_ok;
  rows.push(fb === true ? r("feedback", "ok", "유효") : fb === false ? r("feedback", "crit", "차량 피드백 없음") : r("feedback", "unknown", "모름"));
  const cannot = inp.monitor ? " · 이 화면으로 멈출 수 없음" : "";
  const standing = st?.mode === "PAUSED" || st?.mode === "STOPPED" ? (st.stopped_confirmed === false ? "정지 확인 기다리는 중" : "") : "";
  if (!st || st.mode === null || st.mode === undefined) rows.push(r("stop", "unknown", `모름${cannot}`));
  else if (st.mode === "STOPPED") rows.push(r("stop", "crit", `정지됨 · 재무장 필요${cannot}`, [stopKind(st.stop_kind).label, standing].filter(Boolean).join(" · ")));
  else if (st.stop_kind === "preempt" || st.stop_kind === "safety_preempt") rows.push(r("stop", "crit", `안전 선점${cannot}`, standing));
  else rows.push(r("stop", "ok", `정지 아님${cannot}`, standing));
  const pm = st?.permit;
  if (!pm || !Number.isFinite(pm.limit_mps)) rows.push(r("permit", "unknown", "판정 없음"));
  else rows.push(r("permit", pm.limit_mps === 0 ? "warn" : "ok", permitText(pm, st.teleop_forbidden_reason)));
  const dmg = st?.scene?.damaged;
  if (!st.scene) rows.push(r("scene", "off", "차량이 주지 않음"));
  else rows.push(dmg ? r("scene", "crit", `장면 손상 · ${dmg.reason}`) : r("scene", "ok", "손상 없음"));
  return rows.concat(shownRows(inp));
}

// shown, never raised (from the last state the page holds, labelled as such by their values)
function shownRows(inp) {
  const st = inp.state;
  const rows = [];
  rows.push(r("lease", "off", leaseValue(inp)));
  rows.push(r("slam", "off", slamLabel(st).text));
  rows.push(r("freshness", "off", freshnessValue(st)));
  rows.push(sessionRow(inp));
  rows.push(queueRow(inp.queue));
  return rows;
}

// The alarm book (plan D3): a row's warn/crit that lasts opens an alarm at its first sighting; ok that lasts closes it
// as recovered, off closes it as no longer applying, unknown leaves it and breaks the count.
export class AlarmBook {
  constructor() {
    this.alarms = []; // newest first
    this.streaks = new Map(); // key -> {kind: "bad"|"ok", since, sinceUnix}
    this.lastCall = null;
    this.epoch = null;
    this.nextId = 1;
    this.unseenCrit = 0;
    this.version = 0;
  }

  update(rows, nowMs, nowUnix, { epoch = null, viewing = false } = {}) {
    if (this.lastCall !== null && nowMs - this.lastCall > GAP_MS) this.streaks.clear(); // not seen in between
    this.lastCall = nowMs;
    if (epoch !== null && this.epoch !== null && epoch !== this.epoch) {
      for (const a of this.#open()) this.#close(a, nowUnix, "새 세션", viewing);
      this.streaks.clear();
    }
    if (epoch !== null) this.epoch = epoch;
    for (const row of rows) this.#row(row, nowMs, nowUnix, viewing);
    // seen while open and critical: not unseen when it closes elsewhere (a warning seen is not a critical one seen)
    if (viewing) {
      for (const a of this.#open()) {
        if (a.level === "crit" && !a.seen) {
          a.seen = true;
          this.version += 1;
        }
      }
    }
  }

  markSeen() {
    if (this.unseenCrit !== 0) {
      this.unseenCrit = 0;
      this.version += 1;
    }
  }

  toStorage() {
    return { v: 1, alarms: this.alarms, nextId: this.nextId, unseenCrit: this.unseenCrit };
  }

  // a reopened page did not see the time it was closed: what was open is closed as unseen and judged again
  static fromStorage(obj, nowUnix) {
    const b = new AlarmBook();
    if (!obj || obj.v !== 1 || !Array.isArray(obj.alarms)) return b;
    b.alarms = obj.alarms.filter((a) => a && typeof a.key === "string").slice(0, KEEP).map((a) => ({ ...a }));
    b.nextId = Number.isInteger(obj.nextId) ? obj.nextId : b.alarms.length + 1;
    b.unseenCrit = Number.isInteger(obj.unseenCrit) && obj.unseenCrit > 0 ? obj.unseenCrit : 0;
    for (const a of b.alarms) {
      if (a.cleared_unix === null) {
        a.cleared_unix = nowUnix;
        a.cleared_note = "화면 다시 열림 — 그 사이 모름";
        if (a.level === "crit" && !a.seen) b.unseenCrit += 1; // never seen: the dot carries it over the reload
      }
    }
    return b;
  }

  #open() {
    return this.alarms.filter((a) => a.cleared_unix === null);
  }

  #row(row, nowMs, nowUnix, viewing) {
    const open = this.alarms.find((a) => a.key === row.key && a.cleared_unix === null);
    const s = this.streaks.get(row.key);
    if (row.level === "warn" || row.level === "crit") {
      if (!s || s.kind !== "bad") this.streaks.set(row.key, { kind: "bad", since: nowMs, sinceUnix: nowUnix });
      const since = this.streaks.get(row.key);
      if (open) {
        if (RANK[row.level] > RANK[open.level]) {
          open.level = row.level;
          open.peak_text = row.value;
          open.seen = false; // the critical level itself has not been seen yet
          this.version += 1;
        }
        if (open.text !== row.value) {
          open.text = row.value;
          this.version += 1;
        }
      } else if (nowMs - since.since >= OPEN_MS) {
        this.alarms.unshift({ id: this.nextId++, key: row.key, label: row.label, level: row.level, text: row.value,
                              peak_text: row.value, opened_unix: since.sinceUnix, cleared_unix: null, cleared_note: null,
                              seen: viewing && row.level === "crit" });
        this.#trim();
        this.version += 1;
      }
    } else if (row.level === "ok") {
      if (!s || s.kind !== "ok") this.streaks.set(row.key, { kind: "ok", since: nowMs, sinceUnix: nowUnix });
      if (open && nowMs - this.streaks.get(row.key).since >= CLEAR_MS) this.#close(open, nowUnix, "복구", viewing);
    } else if (row.level === "off") {
      this.streaks.delete(row.key);
      if (open) this.#close(open, nowUnix, /세션 종료/.test(row.value) ? "관측 종료" : "해당 없어짐", viewing);
    } else {
      this.streaks.delete(row.key); // unknown: not better, not worse
    }
  }

  #close(a, nowUnix, note, viewing) {
    a.cleared_unix = nowUnix;
    a.cleared_note = note;
    if (a.level === "crit" && !viewing && !a.seen) this.unseenCrit += 1;
    this.version += 1;
  }

  #trim() {
    while (this.alarms.length > KEEP) {
      const i = this.alarms.findLastIndex((a) => a.cleared_unix !== null);
      if (i < 0) break;
      this.alarms.splice(i, 1);
    }
  }
}

export function badge(book) {
  // the dot shows beside an open warning too: a critical alarm that came and went unseen is never hidden
  const open = book.alarms.filter((a) => a.cleared_unix === null);
  const level = open.length ? (open.some((a) => a.level === "crit") ? "crit" : "warn") : null;
  return { count: open.length, level, dot: book.unseenCrit > 0 };
}
