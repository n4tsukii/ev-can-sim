import itertools
import time

import pytest

from evcan import BMS, Codec, UdsClient, UdsNegativeResponse, crc8_j1850
from evcan.isotp import segment
from evcan.uds import EXTENDED_SESSION, NRC_NOT_IN_ACTIVE_SESSION, NRC_REQUEST_OUT_OF_RANGE, dtc_to_str
from run_sim import run

_ids = itertools.count()


def channel():
    return f"test_{next(_ids)}"     # isolate each test on its own virtual bus


@pytest.fixture(scope="module")
def codec():
    return Codec()


# ---------- codec / E2E ----------
def test_crc8_check_value():
    assert crc8_j1850(b"123456789") == 0x4B


def test_encode_decode_roundtrip(codec):
    data = codec.encode("BMS_Status", {"SOC": 75.5, "PackVoltage": 391.2, "PackCurrent": -42.3,
                                       "MaxCellTemp": 31, "BMS_State": 2}, counter=7)
    assert codec.checksum_ok(data)
    sig = codec.decode(0x100, data)
    assert sig["SOC"] == 75.5 and sig["PackCurrent"] == pytest.approx(-42.3)
    assert sig["MaxCellTemp"] == 31 and sig["BMS_State"] == 2 and sig["AliveCounter"] == 7


def test_corrupted_frame_detected(codec):
    data = bytearray(codec.encode("VCU_Drive", {"VehicleSpeed": 50}, counter=0))
    data[0] ^= 0x01
    assert not codec.checksum_ok(bytes(data))


def test_cycle_times_from_dbc(codec):
    assert codec.cycle_time_s("VCU_Drive") == 0.02
    assert codec.cycle_time_s("BMS_Status") == 0.1


# ---------- ISO-TP ----------
def test_isotp_single_frame():
    frames = segment(b"\x22\xF1\x90")
    assert frames == [bytes([0x03, 0x22, 0xF1, 0x90, 0xCC, 0xCC, 0xCC, 0xCC])]


def test_isotp_multi_frame_layout():
    payload = bytes(range(20))
    frames = segment(payload)
    assert frames[0][:2] == bytes([0x10, 20])
    assert [f[0] for f in frames[1:]] == [0x21, 0x22]
    assert b"".join([frames[0][2:]] + [f[1:] for f in frames[1:]])[:20] == payload


def test_dtc_format():
    assert dtc_to_str(0x0A7E00) == "P0A7E-00"
    assert dtc_to_str(0xC10000) == "U0100-00"


# ---------- UDS against a running BMS ----------
@pytest.fixture
def bms_and_tester(codec):
    ch = channel()
    bms = BMS(codec, ch)
    bms.start()
    tester = UdsClient(ch)
    yield bms, tester
    tester.close()
    bms.stop()


def test_read_vin_multiframe(bms_and_tester):
    _, tester = bms_and_tester
    assert tester.read_did(0xF190) == b"RLLVF8EV0TN000123"


def test_unknown_did_negative_response(bms_and_tester):
    _, tester = bms_and_tester
    with pytest.raises(UdsNegativeResponse) as e:
        tester.read_did(0x1234)
    assert e.value.nrc == NRC_REQUEST_OUT_OF_RANGE


def test_clear_dtc_needs_extended_session(bms_and_tester):
    bms, tester = bms_and_tester
    bms.dtcs[0x0A7E00] = 0x08
    with pytest.raises(UdsNegativeResponse) as e:
        tester.clear_dtcs()
    assert e.value.nrc == NRC_NOT_IN_ACTIVE_SESSION
    tester.change_session(EXTENDED_SESSION)
    tester.clear_dtcs()
    assert tester.read_dtcs() == []


# ---------- end-to-end scenarios ----------
def test_normal_has_no_events():
    mon, _ = run("normal", 1.5, channel=channel(), out_dir=None, verbose=False)
    assert mon.events == []
    assert mon.measured_cycle_ms(0x200) == pytest.approx(20, rel=0.1)


def test_overheat_sets_fault_and_dtc():
    mon, diag = run("overheat", 4.5, channel=channel(), out_dir=None, verbose=False)
    assert {"TEMP_WARNING", "BMS_FAULT"} <= mon.kinds()
    assert ("P0A7E-00", 0x09) in diag["dtcs"]


def test_brake_loss_detected_as_timeout():
    mon, _ = run("brake_loss", 1.5, channel=channel(), out_dir=None, verbose=False)
    assert any(e.kind == "TIMEOUT" and e.message == "BRK_Status" for e in mon.events)


def test_bad_checksum_detected():
    mon, _ = run("bad_checksum", 1.5, channel=channel(), out_dir=None, verbose=False)
    assert sum(e.kind == "CHECKSUM_ERROR" for e in mon.events) == 3


def test_logs_written(tmp_path):
    run("normal", 0.5, channel=channel(), out_dir=str(tmp_path), verbose=False)
    for f in ("trace.asc", "signals.csv", "events.csv"):
        assert (tmp_path / f).stat().st_size > 0
