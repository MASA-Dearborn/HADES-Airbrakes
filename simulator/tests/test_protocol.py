"""Framing/CRC tests for the HIL wire protocol."""

import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hades_hil import protocol
from hades_hil.protocol import FrameParser, Status


def test_sensor_roundtrip():
    frame = protocol.make_sensor(123456, 0.1, 0.2, -9.81, 0.01, 0.02, 0.03,
                                 1013.2, 21.5, flags=protocol.FLAG_IMU | protocol.FLAG_BARO)
    parser = FrameParser()
    packets = parser.feed(frame)
    assert len(packets) == 1
    pkt_type, payload = packets[0]
    assert pkt_type == protocol.PKT_SENSOR
    fields = struct.unpack(protocol.SENSOR_FMT, payload)
    assert fields[0] == 123456
    assert abs(fields[3] - (-9.81)) < 1e-5
    assert fields[12] == protocol.FLAG_IMU | protocol.FLAG_BARO


def test_status_roundtrip():
    payload = struct.pack(protocol.STATUS_FMT, 999, 2,
                          1500.0, 250.0, 4.2, 3600.0, 552.0, 7.5, 6.9,
                          1.25, 0.75, -42.0)
    frame = protocol.make_frame(protocol.PKT_STATUS, payload)
    parser = FrameParser()
    [(pkt_type, raw)] = parser.feed(frame)
    status = Status.unpack(raw)
    assert pkt_type == protocol.PKT_STATUS
    assert status.phase_name == "COASTING"
    assert abs(status.actual_pos_cm - 6.9) < 1e-5
    assert abs(status.kf_p00 - 1.25) < 1e-5
    assert abs(status.act_duty_pct - (-42.0)) < 1e-5


def test_parser_skips_ascii_debug_text():
    lines = []
    parser = FrameParser(text_callback=lines.append)
    frame = protocol.make_init(1010.0, 20.0)
    stream = b"HIL: waiting for INIT from host...\r\n" + frame + b"Base pressure: 1010.00\n"
    packets = parser.feed(stream)
    assert [t for t, _ in packets] == [protocol.PKT_INIT]
    assert lines == ["HIL: waiting for INIT from host...", "Base pressure: 1010.00"]


def test_corrupt_crc_dropped():
    frame = bytearray(protocol.make_init(1000.0, 15.0))
    frame[-1] ^= 0xFF
    assert FrameParser().feed(bytes(frame)) == []


def test_split_frame_across_feeds():
    frame = protocol.make_sensor(42, 0, 0, -9.81, 0, 0, 0, 1013.25, 15.0)
    parser = FrameParser()
    assert parser.feed(frame[:10]) == []
    [(pkt_type, _)] = parser.feed(frame[10:])
    assert pkt_type == protocol.PKT_SENSOR


def test_crc8_matches_reference():
    # CRC-8/SMBUS-style poly 0x07, init 0x00: crc8("123456789") = 0xF4
    assert protocol.crc8(b"123456789") == 0xF4
