// Media connection frames (contract 7): uint32 big-endian header length, UTF-8 JSON header, body.

export function parseFrame(buffer) {
  const view = new DataView(buffer);
  const n = view.getUint32(0, false);
  const head = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, n)));
  return { head, body: buffer.slice(4 + n) };
}

export function decodeScan(head, body) {
  const ranges = new Float32Array(body.slice(0, head.count * 4));
  const out = [];
  const p = head.pose;
  for (let i = 0; i < ranges.length; i += 1) {
    const r = ranges[i];
    if (!Number.isFinite(r)) continue; // NaN = no return (unknown), never "free"
    const a = p.yaw_rad + head.angle_min_rad + i * head.angle_increment_rad;
    out.push(p.x_m + r * Math.cos(a), p.y_m + r * Math.sin(a));
  }
  return out;
}
