"""Decode a CAN log (candump .log, Vector .asc, ...) with the project DBC and verify E2E protection.

    python tools/decode_log.py ../bms-firmware-c/bms_can.log
"""
from __future__ import annotations

import sys
from pathlib import Path

import can

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from evcan import Codec  # noqa: E402


def main(path: str, every: int = 10) -> int:
    codec = Codec()
    known = {m.frame_id for m in codec.db.messages}
    last_cnt: dict[int, int] = {}
    errors = total = 0
    prev_state = None
    for i, msg in enumerate(can.LogReader(path)):
        if msg.arbitration_id not in known:
            continue
        total += 1
        name = codec.message(msg.arbitration_id).name
        if not codec.checksum_ok(msg.data):
            errors += 1
            print(f"{msg.timestamp:9.3f}  {name}  CHECKSUM ERROR  {msg.data.hex(' ')}")
            continue
        sig = codec.decode(msg.arbitration_id, msg.data)
        cnt = int(sig["AliveCounter"])
        if msg.arbitration_id in last_cnt and cnt != (last_cnt[msg.arbitration_id] + 1) & 0xF:
            errors += 1
            print(f"{msg.timestamp:9.3f}  {name}  COUNTER JUMP {last_cnt[msg.arbitration_id]} -> {cnt}")
        last_cnt[msg.arbitration_id] = cnt
        state = sig.get("BMS_State")
        if i % every == 0 or state != prev_state:
            body = "  ".join(f"{k}={codec.choice_name(name, k, v) if k == 'BMS_State' else round(v, 3)}"
                             for k, v in sig.items() if k not in ("AliveCounter", "Checksum"))
            print(f"{msg.timestamp:9.3f}  {name}  {body}")
        prev_state = state
    print(f"\n{total} frames decoded, {errors} E2E errors")
    return 1 if errors else 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
