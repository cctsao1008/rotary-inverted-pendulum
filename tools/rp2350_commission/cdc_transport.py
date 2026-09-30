from __future__ import annotations

from collections import deque
import threading
import time

import serial
from serial.tools import list_ports

from protocol import PID, VID


class CdcTransport:
    def __init__(self, *, port: str | None = None, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self._ser: serial.Serial | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._lines: deque[str] = deque(maxlen=4096)
        self._lock = threading.Lock()

    @staticmethod
    def enumerate() -> list[str]:
        ports: list[str] = []
        for info in list_ports.comports():
            if info.vid == VID and info.pid == PID:
                ports.append(info.device)
        return ports

    def open(self) -> None:
        if self._ser is not None:
            return
        port = self.port
        if port is None:
            candidates = self.enumerate()
            if not candidates:
                raise RuntimeError(f"RP2350 CDC {VID:04X}:{PID:04X} not found")
            port = candidates[0]
        self._ser = serial.Serial(port, self.baudrate, timeout=0.1)
        self.port = port
        self._stop.clear()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=0.5)
            self._reader = None
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self) -> "CdcTransport":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _reader_loop(self) -> None:
        assert self._ser is not None
        while not self._stop.is_set():
            raw = self._ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            with self._lock:
                self._lines.append(line)

    def command(self, text: str, *, wait_s: float = 0.15) -> list[str]:
        if self._ser is None:
            raise RuntimeError("CDC port is not open")
        with self._lock:
            self._lines.clear()
        self._ser.write((text.rstrip() + "\r\n").encode("ascii"))
        self._ser.flush()
        time.sleep(wait_s)
        return self.drain_lines()

    def drain_lines(self) -> list[str]:
        with self._lock:
            result = list(self._lines)
            self._lines.clear()
        return result
