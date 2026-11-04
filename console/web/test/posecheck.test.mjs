import { test } from "node:test";
import assert from "node:assert/strict";
import { poseCheckOf, poseBand, poseZones } from "../js/posecheck.js";

// docs/plans/2026-11-02-unconfirmed-runner.md 4: the state's pose_check while the start is not confirmed

test("no field, or a confirmed start, shows nothing", () => {
  assert.equal(poseCheckOf({}), null);
  assert.equal(poseCheckOf({ pose_check: { status: "confirmed" } }), null);
  assert.equal(poseBand(null), null);
});

test("unconfirmed says what is locked and what the operator must look at, with the stroke left", () => {
  const pc = { status: "unconfirmed", stroke: { state: "ready", left_m: 0.932, total_left_m: 2.932 }, zones: [] };
  const band = poseBand(poseCheckOf({ pose_check: pc }));
  assert.match(band.text, /^확인 전 위치 · 전진·포크 잠김 · 뒤쪽 낮은 물체는 보이지 않습니다 · 차 뒤를 직접 확인하세요/);
  assert.match(band.text, /곧은 후진 0\.93 m 남음/);
  assert.match(band.text, /전체 2\.93 m/);
  assert.equal(band.tone, "warn");
});

test("a closed or spent stroke says why the reverse does not move", () => {
  const closed = poseBand({ status: "unconfirmed", stroke: { state: "closed", left_m: 0.05, total_left_m: 2.0 } });
  assert.match(closed.text, /차가 선 뒤 손을 뗐다가 다시 누르세요/);
  const spent = poseBand({ status: "unconfirmed", stroke: { state: "spent", left_m: 0.0, total_left_m: 0.0 } });
  assert.match(spent.text, /후진 거리를 다 썼습니다/);
});

test("checking and an end have their own lines", () => {
  assert.match(poseBand({ status: "checking" }).text, /출발 위치를 확인하는 중입니다/);
  const end = poseBand({ status: "ended:pose_rear_blocked" });
  assert.match(end.text, /뒤에 공간이 없어 출발 위치를 확인할 수 없습니다/);
  assert.equal(end.tone, "bad");
  assert.match(poseBand({ status: "ended:pose_revoked:int" }).text, /권한을 거뒀습니다/);
});

test("the hypothesis zones are polygons in the map, at most the cap", () => {
  const sq = [[0, 0], [1, 0], [1, 1], [0, 1]];
  const pc = { status: "unconfirmed", zones: Array.from({ length: 80 }, () => sq) };
  const zones = poseZones(pc);
  assert.equal(zones.length, 64);
  assert.deepEqual(zones[0], sq);
  assert.deepEqual(poseZones({ status: "unconfirmed", zones: [[[0, 0], [1, "x"]]] }), []); // not drawable: dropped
});
