"""Serial link to the Teensy running the teensy41-hil firmware build."""

import time

import serial
from serial.tools import list_ports

from . import protocol
from .protocol import FrameParser, Status


def autodetect_port() -> str:
    ports = list(list_ports.comports())

    # Teensy USB Serial uses PJRC's vendor ID. Prefer it so Bluetooth COM ports
    # and unrelated USB serial adapters are not selected on Windows.
    teensy = sorted(
        port.device for port in ports
        if port.vid == 0x16C0 and port.pid == 0x0483
    )
    if teensy:
        return teensy[0]

    # Useful fallback for macOS when USB VID/PID metadata is unavailable.
    usb_modem = sorted(
        port.device for port in ports
        if port.device.startswith("/dev/cu.usbmodem")
    )
    if usb_modem:
        return usb_modem[0]

    raise RuntimeError(
        "No Teensy USB serial device found. Plug it in or pass --port "
        "(for example, --port COM7)."
    )


class HilLink:
    def __init__(self, port: str, baud: int = 115200, timeout_s: float = 5.0,
                 text_callback=None):
        self.timeout_s = timeout_s
        self._text_callback = text_callback or (lambda line: print(f"[teensy] {line}"))
        self.parser = FrameParser(text_callback=self._text_callback)
        self.ser = serial.Serial(port, baud, timeout=0.05)

    def close(self):
        self.ser.close()

    def _read_packets(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        return self.parser.feed(data) if data else []

    def handshake(self, base_pressure_hpa: float, base_temp_c: float,
                  timeout_s: float = 30.0):
        """Send INIT until the Teensy replies INIT_ACK.

        The HIL firmware blocks in hilInit() at boot, so this also tolerates
        the Teensy being reset/replugged mid-wait.
        """
        frame = protocol.make_init(base_pressure_hpa, base_temp_c)
        deadline = time.monotonic() + timeout_s
        last_send = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() - last_send >= 0.5:
                self.ser.write(frame)
                last_send = time.monotonic()
            for pkt_type, _payload in self._read_packets():
                if pkt_type == protocol.PKT_INIT_ACK:
                    return
        raise TimeoutError("Teensy did not acknowledge INIT — is the "
                           "teensy41-hil build flashed?")

    def exchange(self, sensor_frame: bytes) -> Status:
        """Send one SENSOR frame and block until its STATUS reply."""
        self.ser.write(sensor_frame)
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            for pkt_type, payload in self._read_packets():
                if pkt_type == protocol.PKT_STATUS:
                    return Status.unpack(payload)
        raise TimeoutError("No STATUS reply from Teensy")
