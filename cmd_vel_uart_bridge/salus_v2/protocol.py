"""UART protocol frames and CRC utilities."""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag

try:
    from crccheck.crc import Crc8DallasMaxim as _Crc8DallasMaxim
except ImportError:  # pragma: no cover - compatibility with older crccheck
    from crccheck.crc import Crc8Maxim as _Crc8DallasMaxim

START_CMD = 0xAA
START_STATUS = 0x55
CMD_LEN = 6
STATUS_LEN = 4
VER_MAJOR = 0x1


class CommandFlags(IntFlag):
    ESTOP = 1 << 0
    DRIVE_EN = 1 << 1
    ALLOW_REVERSE = 1 << 2


class StatusFlags(IntFlag):
    READY = 1 << 0
    FAULT = 1 << 1
    OVERCURRENT = 1 << 2
    REVERSE_REQ = 1 << 3


def crc8_maxim(data: bytes) -> int:
    return _Crc8DallasMaxim.calc(data)


def clamp_int(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(value)))


def int8_to_u8(value: int) -> int:
    v = int(value)
    if v < -128 or v > 127:
        raise ValueError(f"int8 out of range: {v}")
    return v & 0xFF


def u8_to_int8(value: int) -> int:
    v = int(value) & 0xFF
    return v - 256 if v > 127 else v


@dataclass(frozen=True)
class CommandFrame:
    steer: int
    accel: int
    brake: int
    flags: CommandFlags
    version: int = VER_MAJOR

    def pack(self) -> bytes:
        ver_flags = ((self.version & 0x0F) << 4) | (int(self.flags) & 0x0F)
        steer_u8 = int8_to_u8(self.steer)
        accel_u8 = int8_to_u8(self.accel)
        brake_u8 = clamp_int(self.brake, 0, 100) & 0xFF
        payload = bytes([START_CMD, ver_flags, steer_u8, accel_u8, brake_u8])
        crc = crc8_maxim(payload)
        return payload + bytes([crc])

    @staticmethod
    def parse(data: bytes) -> CommandFrame | None:
        if len(data) != CMD_LEN or data[0] != START_CMD:
            return None
        if crc8_maxim(data[: CMD_LEN - 1]) != data[-1]:
            return None
        ver = (data[1] >> 4) & 0x0F
        flags = CommandFlags(data[1] & 0x0F)
        steer = u8_to_int8(data[2])
        accel = u8_to_int8(data[3])
        brake = int(data[4])
        return CommandFrame(steer=steer, accel=accel, brake=brake, flags=flags, version=ver)


@dataclass(frozen=True)
class StatusFrame:
    flags: StatusFlags
    telemetry: int

    def pack(self) -> bytes:
        payload = bytes([START_STATUS, int(self.flags) & 0xFF, self.telemetry & 0xFF])
        crc = crc8_maxim(payload)
        return payload + bytes([crc])

    @staticmethod
    def parse(data: bytes) -> StatusFrame | None:
        if len(data) != STATUS_LEN or data[0] != START_STATUS:
            return None
        if crc8_maxim(data[: STATUS_LEN - 1]) != data[-1]:
            return None
        flags = StatusFlags(data[1])
        telemetry = int(data[2])
        return StatusFrame(flags=flags, telemetry=telemetry)


class StatusFrameParser:
    def __init__(self, max_buffer: int = 1024):
        self._buffer = bytearray()
        self._max_buffer = max_buffer

    def feed(self, data: bytes) -> tuple[list[StatusFrame], int]:
        if not data:
            return [], 0
        self._buffer.extend(data)
        frames: list[StatusFrame] = []
        crc_errors = 0
        i = 0
        while i <= len(self._buffer) - STATUS_LEN:
            if self._buffer[i] != START_STATUS:
                i += 1
                continue
            candidate = bytes(self._buffer[i : i + STATUS_LEN])
            parsed = StatusFrame.parse(candidate)
            if parsed:
                frames.append(parsed)
                del self._buffer[i : i + STATUS_LEN]
                continue
            crc_errors += 1
            del self._buffer[i]
        if len(self._buffer) > self._max_buffer:
            self._buffer = self._buffer[-STATUS_LEN:]
        return frames, crc_errors
