"""Wire protocol — Python mirror of HADES-Airbrakes src/hil.cpp.

Frame: [0xAA][0x55][type][len][payload...][crc8]
crc8: poly 0x07, init 0x00, over type+len+payload.
ASCII debug text from the Teensy may appear between frames; 0xAA never occurs
in 7-bit ASCII so the parser skips it safely and surfaces it as text lines.
"""

from dataclasses import dataclass
import struct

SYNC1 = 0xAA
SYNC2 = 0x55

PKT_INIT = 0x01
PKT_SENSOR = 0x02
PKT_INIT_ACK = 0x81
PKT_STATUS = 0x82

FLAG_IMU = 0x01
FLAG_BARO = 0x02
FLAG_MAG = 0x04

INIT_FMT = "<2f"                 # basePressure_hPa, baseTempC
SENSOR_FMT = "<I11fB"            # simTimeUs, ax..az, gx..gz, hpa, tempC, mx..mz, flags
STATUS_FMT = "<IB10f"            # simTimeUs, phase, estH, estV, tilt, estApogee, apogeeErr, targetPos, actualPos, kfP00, kfP11, actDutyPct

PHASE_NAMES = ["IDLE", "LAUNCHED", "COASTING", "APOGEE", "DESCENT", "FAULT"]


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def make_frame(pkt_type: int, payload: bytes) -> bytes:
    body = bytes([pkt_type, len(payload)]) + payload
    return bytes([SYNC1, SYNC2]) + body + bytes([crc8(body)])


def make_init(base_pressure_hpa: float, base_temp_c: float) -> bytes:
    return make_frame(PKT_INIT, struct.pack(INIT_FMT, base_pressure_hpa, base_temp_c))


def make_sensor(sim_time_us, ax, ay, az, gx, gy, gz, hpa, temp_c,
                mx=0.0, my=0.0, mz=0.0, flags=FLAG_IMU) -> bytes:
    payload = struct.pack(SENSOR_FMT, sim_time_us & 0xFFFFFFFF,
                          ax, ay, az, gx, gy, gz, hpa, temp_c, mx, my, mz, flags)
    return make_frame(PKT_SENSOR, payload)


@dataclass
class Status:
    sim_time_us: int
    phase: int
    est_h: float
    est_v: float
    tilt_deg: float
    est_apogee_m: float
    apogee_err_m: float
    target_pos_cm: float
    actual_pos_cm: float
    kf_p00: float
    kf_p11: float
    act_duty_pct: float

    @property
    def phase_name(self) -> str:
        return PHASE_NAMES[self.phase] if self.phase < len(PHASE_NAMES) else "?"

    @classmethod
    def unpack(cls, payload: bytes) -> "Status":
        return cls(*struct.unpack(STATUS_FMT, payload))


class FrameParser:
    """Byte-stream parser; same state machine as hil.cpp rxFeed().

    feed() returns a list of (type, payload) tuples.  Non-frame printable
    bytes are buffered and emitted line-by-line through text_callback —
    these are the Teensy's Serial.println() debug messages.
    """

    def __init__(self, text_callback=None):
        self._state = "SYNC1"
        self._type = 0
        self._len = 0
        self._buf = bytearray()
        self._text = bytearray()
        self._text_callback = text_callback

    def _emit_text(self, byte: int):
        if byte == 0x0A:
            if self._text and self._text_callback:
                self._text_callback(self._text.decode("ascii", "replace").rstrip())
            self._text.clear()
        elif 0x20 <= byte < 0x7F:
            self._text.append(byte)

    def feed(self, data: bytes):
        packets = []
        for byte in data:
            if self._state == "SYNC1":
                if byte == SYNC1:
                    self._state = "SYNC2"
                else:
                    self._emit_text(byte)
            elif self._state == "SYNC2":
                self._state = "TYPE" if byte == SYNC2 else "SYNC1"
            elif self._state == "TYPE":
                self._type = byte
                self._state = "LEN"
            elif self._state == "LEN":
                self._len = byte
                self._buf.clear()
                self._state = "CRC" if byte == 0 else "PAYLOAD"
            elif self._state == "PAYLOAD":
                self._buf.append(byte)
                if len(self._buf) >= self._len:
                    self._state = "CRC"
            elif self._state == "CRC":
                self._state = "SYNC1"
                body = bytes([self._type, self._len]) + bytes(self._buf)
                if crc8(body) == byte:
                    packets.append((self._type, bytes(self._buf)))
        return packets
