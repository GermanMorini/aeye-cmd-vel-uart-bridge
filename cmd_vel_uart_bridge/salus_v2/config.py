"""Configuration loading via environment variables."""
from __future__ import annotations

from typing import Set

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="SALUS_", case_sensitive=False)

    serial_port: str = "/dev/serial0"
    baudrate: int = 460800
    serial_timeout_s: float = 0.001
    gpio_relay: int = 8
    gpio_off_list: str = "8,7,16,24,25"
    status_timeout_s: float = 0.5
    cmd_tx_period_s: float = 0.010
    direction_change_delay_s: float = 2.0
    accel_ramp_time_s: float = 0.20
    inactivity_brake_s: float = 0.4
    resume_min_interval_s: float = 0.25
    resume_hits_required: int = 2
    ws_host: str = "0.0.0.0"
    ws_port: int = 8765
    ws_path: str = "/controls"
    gpio_mock: bool = False

    def gpio_off_pins(self) -> Set[int]:
        pins: Set[int] = set()
        for token in self.gpio_off_list.split(","):
            token = token.strip()
            if not token:
                continue
            try:
                pins.add(int(token))
            except ValueError:
                continue
        pins.add(int(self.gpio_relay))
        return pins
