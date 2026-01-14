"""Shared state models for controller and IO."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Targets:
    steer: int = 0
    accel: int = 0
    brake: int = 0


@dataclass
class Esp32Status:
    ready: bool = False
    fault: bool = False
    overcurrent: bool = False
    reverse_req: bool = False
    telemetry: int = 0
    last_packet_ts: float = 0.0
    packets_ok: int = 0
    crc_errors: int = 0
