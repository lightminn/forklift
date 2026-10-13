import { test } from "node:test";
import assert from "node:assert/strict";
import { parseFrame } from "../js/media.js";

test("parses length-prefixed header and body", () => {
  const head = new TextEncoder().encode(JSON.stringify({ t: "camera", frame_id: 3 }));
  const buf = new Uint8Array(4 + head.length + 2);
  new DataView(buf.buffer).setUint32(0, head.length);
  buf.set(head, 4); buf.set([0xff, 0xd8], 4 + head.length);
  const f = parseFrame(buf.buffer);
  assert.equal(f.head.frame_id, 3);
  assert.deepEqual([...new Uint8Array(f.body)], [0xff, 0xd8]);
});
