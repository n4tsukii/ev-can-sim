"""Run the EV CAN network simulation with an optional fault scenario, then a UDS diagnostic session.

    python run_sim.py --scenario overheat --duration 6
"""
from __future__ import annotations

import argparse
import time

from evcan import BMS, VCU, BrakeECU, BusMonitor, Codec, UdsClient, UdsNegativeResponse
from evcan.uds import EXTENDED_SESSION

SCENARIOS = ("normal", "overheat", "brake_loss", "bad_checksum")


def run(scenario: str = "normal", duration: float = 6.0, channel: str = "ev_can", out_dir: str = "logs",
        verbose: bool = True) -> tuple[BusMonitor, dict]:
    codec = Codec()
    monitor = BusMonitor(codec, channel, out_dir=out_dir)
    bms, vcu, brk = BMS(codec, channel), VCU(codec, channel), BrakeECU(codec, channel)
    nodes = [bms, vcu, brk]

    monitor.start()
    for n in nodes:
        n.start()

    t_inject = duration / 3
    injected = False
    t_start = time.time()
    while time.time() - t_start < duration:
        if not injected and time.time() - t_start >= t_inject:
            injected = True
            if scenario == "overheat":
                bms.inject_overheat()
            elif scenario == "brake_loss":
                brk.tx_enabled = False
            elif scenario == "bad_checksum":
                bms.corrupt_next = 3
        time.sleep(0.05)

    diag = uds_session(channel, verbose)

    for n in nodes:
        n.stop()
    monitor.stop()

    if verbose:
        print(f"\n=== Scenario '{scenario}' ({duration:.1f} s) ===")
        print("Cycle times (DBC vs measured):")
        for m in codec.db.messages:
            meas = monitor.measured_cycle_ms(m.frame_id)
            print(f"  0x{m.frame_id:03X} {m.name:<11} {m.cycle_time:>4} ms  ->  "
                  f"{meas:6.1f} ms  ({monitor.stats[m.frame_id]['count']} frames)" if meas else "")
        print("Events:")
        for e in monitor.events or []:
            print(f"  t={e.t:6.3f}s  {e.kind:<15} {e.message:<11} {e.detail}")
        if not monitor.events:
            print("  (none)")
        if out_dir:
            print(f"Logs written to {out_dir}/ (trace.asc, signals.csv, events.csv)")
    return monitor, diag


def uds_session(channel: str, verbose: bool) -> dict:
    client = UdsClient(channel)
    out = {}
    try:
        out["vin"] = client.read_did(0xF190).decode()
        out["soc"] = client.read_did(0x0101)[0] / 2
        out["temp"] = client.read_did(0x0102)[0] - 40
        out["dtcs"] = client.read_dtcs()
        try:
            client.clear_dtcs()
            out["clear_default"] = "accepted"
        except UdsNegativeResponse as e:
            out["clear_default"] = str(e)
        client.change_session(EXTENDED_SESSION)
        client.clear_dtcs()
        out["dtcs_after_clear"] = client.read_dtcs()
    finally:
        client.close()
    if verbose:
        print("\n=== UDS session (tester 0x7E0 -> BMS 0x7E8) ===")
        print(f"  22 F190 VIN        : {out['vin']}")
        print(f"  22 0101 SOC        : {out['soc']} %")
        print(f"  22 0102 MaxCellTemp: {out['temp']} degC")
        print(f"  19 02 FF DTCs      : {[f'{c} status=0x{s:02X}' for c, s in out['dtcs']] or 'none'}")
        print(f"  14 FFFFFF (default): {out['clear_default']}")
        print(f"  10 03 + 14 FFFFFF  : cleared -> {[c for c, _ in out['dtcs_after_clear']] or 'none'}")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=SCENARIOS, default="normal")
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--out", default="logs")
    args = ap.parse_args()
    run(args.scenario, args.duration, out_dir=args.out)
