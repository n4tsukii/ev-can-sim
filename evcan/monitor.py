"""Bus monitor / gateway: decodes traffic, checks E2E protection and timeouts, logs to ASC + CSV."""
from __future__ import annotations

import csv
import threading
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import can

from .codec import Codec
from .ecu import BMS_FAULT

TIMEOUT_FACTOR = 3          # message considered lost after 3 missed cycles
TEMP_WARNING_C = 55


@dataclass
class Event:
    t: float
    kind: str
    message: str
    detail: str = ""


class BusMonitor:
    def __init__(self, codec: Codec, channel: str, interface: str = "virtual", out_dir: str | Path | None = None):
        self.codec = codec
        self.bus = can.Bus(interface=interface, channel=channel)
        self.out_dir = Path(out_dir) if out_dir else None
        self.events: list[Event] = []
        self.rows: list[tuple] = []
        self.stats = defaultdict(lambda: {"count": 0, "first": None, "last": None})
        self.last_counter: dict[int, int] = {}
        self.last_seen: dict[int, float] = {}
        self.timed_out: set[int] = set()
        self.flags: set[str] = set()
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._watchdog = threading.Thread(target=self._watch, daemon=True)
        self._asc = None
        self._notifier = None
        self.t0 = time.time()
        self.known = {m.frame_id: m for m in codec.db.messages}

    def _event(self, kind, message, detail=""):
        self.events.append(Event(round(time.time() - self.t0, 3), kind, message, detail))

    def on_message(self, msg: can.Message):
        fid = msg.arbitration_id
        with self.lock:
            st = self.stats[fid]
            st["count"] += 1
            st["first"] = st["first"] or msg.timestamp
            st["last"] = msg.timestamp
            if fid not in self.known:
                return                              # e.g. diagnostic frames: traced, not decoded
            name = self.known[fid].name
            self.last_seen[fid] = time.time()
            if fid in self.timed_out:
                self.timed_out.discard(fid)
                self._event("RECOVERED", name)
            if not self.codec.checksum_ok(msg.data):
                self._event("CHECKSUM_ERROR", name, msg.data.hex(" "))
                return                              # never trust a corrupted frame
            sig = self.codec.decode(fid, msg.data)
            cnt = int(sig["AliveCounter"])
            prev = self.last_counter.get(fid)
            if prev is not None and cnt != (prev + 1) & 0x0F:
                self._event("COUNTER_JUMP", name, f"{prev} -> {cnt}")
            self.last_counter[fid] = cnt
            t_rel = round(msg.timestamp - self.t0, 3)
            for k, v in sig.items():
                if k not in ("AliveCounter", "Checksum"):
                    self.rows.append((t_rel, name, k, v))
            self._rules(name, sig)

    def _rules(self, name, sig):
        if name != "BMS_Status":
            return
        if sig["MaxCellTemp"] >= TEMP_WARNING_C and "temp" not in self.flags:
            self.flags.add("temp")
            self._event("TEMP_WARNING", name, f"MaxCellTemp={sig['MaxCellTemp']} degC")
        if sig["BMS_State"] == BMS_FAULT and "fault" not in self.flags:
            self.flags.add("fault")
            self._event("BMS_FAULT", name, "BMS entered FAULT state")

    def _watch(self):
        while not self._stop.wait(0.01):
            now = time.time()
            with self.lock:
                for fid, seen in self.last_seen.items():
                    limit = self.known[fid].cycle_time / 1000 * TIMEOUT_FACTOR
                    if fid not in self.timed_out and now - seen > limit:
                        self.timed_out.add(fid)
                        self._event("TIMEOUT", self.known[fid].name, f"no frame for > {limit * 1000:.0f} ms")

    def start(self):
        self.t0 = time.time()
        listeners = [self.on_message]
        if self.out_dir:
            self.out_dir.mkdir(parents=True, exist_ok=True)
            self._asc = can.ASCWriter(self.out_dir / "trace.asc")
            listeners.append(self._asc)
        self._notifier = can.Notifier(self.bus, listeners, timeout=0.05)
        self._watchdog.start()

    def stop(self):
        self._stop.set()
        self._watchdog.join(timeout=1)
        self._notifier.stop()
        if self._asc:
            self._asc.stop()
        self.bus.shutdown()
        if self.out_dir:
            with open(self.out_dir / "signals.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "message", "signal", "value"])
                w.writerows(self.rows)
            with open(self.out_dir / "events.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "event", "message", "detail"])
                w.writerows((e.t, e.kind, e.message, e.detail) for e in self.events)

    def measured_cycle_ms(self, fid: int) -> float | None:
        st = self.stats.get(fid)
        if not st or st["count"] < 2:
            return None
        return (st["last"] - st["first"]) / (st["count"] - 1) * 1000

    def kinds(self) -> set[str]:
        return {e.kind for e in self.events}
