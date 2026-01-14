"""Core control logic: reverse policy, ramps, failsafe."""
from __future__ import annotations

import logging
import time

from cmd_vel_uart_bridge.salus_v2.config import Settings
from cmd_vel_uart_bridge.salus_v2.gpio import Relay
from cmd_vel_uart_bridge.salus_v2.models import Esp32Status, Targets
from cmd_vel_uart_bridge.salus_v2.protocol import (
    CommandFlags,
    CommandFrame,
    StatusFlags,
    StatusFrame,
    clamp_int,
)

LOG = logging.getLogger(__name__)


class Controller:
    def __init__(self, settings: Settings, relay: Relay):
        self.settings = settings
        self.relay = relay
        self.targets = Targets()
        self.status = Esp32Status()

        self.drive_enabled = True
        self.allow_reverse = False
        self.estop = False

        self.auto_grant_reverse = True
        self.allow_reverse_manual = False

        self._ramp_value = 0.0
        self._last_ramp_update: float | None = None
        self._direction_block_until = 0.0
        self._last_accel_sign = 0
        self._status_timeout_logged = False
        self.last_command: CommandFrame | None = None
        self.last_command_ts = 0.0

    def set_targets(self, steer: int | None = None, accel: int | None = None, brake: int | None = None) -> None:
        if steer is not None:
            self.targets.steer = int(steer)
        if accel is not None:
            self.targets.accel = int(accel)
        if brake is not None:
            self.targets.brake = int(brake)

    def set_drive_enabled(self, enabled: bool) -> None:
        self.drive_enabled = bool(enabled)

    def set_estop(self, enabled: bool) -> None:
        self.estop = bool(enabled)
        if self.estop:
            self.drive_enabled = False

    def set_reverse_auto(self) -> None:
        self.allow_reverse_manual = False
        self.auto_grant_reverse = True
        self.allow_reverse = False
        self.relay.set(False)

    def set_reverse_manual(self, allow: bool) -> None:
        self.allow_reverse_manual = True
        self.auto_grant_reverse = False
        self.allow_reverse = bool(allow)
        self.relay.set(self.allow_reverse)

    def mark_crc_error(self, count: int = 1) -> None:
        self.status.crc_errors += int(count)

    def update_status(self, frame: StatusFrame, now: float | None = None) -> None:
        now = time.time() if now is None else now
        prev_ready = self.status.ready
        prev_fault = self.status.fault
        prev_overcurrent = self.status.overcurrent
        prev_reverse_req = self.status.reverse_req
        prev_telemetry = self.status.telemetry

        self.status.ready = bool(frame.flags & StatusFlags.READY)
        self.status.fault = bool(frame.flags & StatusFlags.FAULT)
        self.status.overcurrent = bool(frame.flags & StatusFlags.OVERCURRENT)
        self.status.reverse_req = bool(frame.flags & StatusFlags.REVERSE_REQ)
        self.status.telemetry = frame.telemetry
        self.status.last_packet_ts = now
        self.status.packets_ok += 1
        self._status_timeout_logged = False

        if self.status.ready and not prev_ready:
            LOG.info("ESP32 READY")
        elif not self.status.ready and prev_ready:
            LOG.warning("ESP32 NOT_READY")

        if self.status.fault and not prev_fault:
            LOG.error("FAULT reportado por ESP32 -> estop preventivo")
            self.set_estop(True)
        elif not self.status.fault and prev_fault:
            LOG.info("FAULT limpiado por ESP32")

        if self.status.overcurrent and not prev_overcurrent:
            LOG.error("OVERCURRENT -> aplicando freno")
            self.set_estop(True)
            self.targets.accel = 0
            self.targets.brake = 100
        elif not self.status.overcurrent and prev_overcurrent:
            LOG.info("OVERCURRENT liberado")

        if self.status.reverse_req != prev_reverse_req:
            LOG.info("REVERSE_REQ -> %s", self.status.reverse_req)

        if self.status.telemetry != prev_telemetry:
            LOG.info("telemetry=%s", self.status.telemetry)

    def enforce_status_timeout(self, now: float | None = None) -> None:
        now = time.time() if now is None else now
        last = self.status.last_packet_ts
        if last == 0.0:
            return
        elapsed = now - last
        if elapsed <= self.settings.status_timeout_s:
            return
        if not self._status_timeout_logged:
            LOG.warning("%.0f ms sin estado -> estado seguro", elapsed * 1000)
            self._status_timeout_logged = True
        self.drive_enabled = False
        self.allow_reverse = False
        self.allow_reverse_manual = False
        self.auto_grant_reverse = True
        self.estop = True
        self.targets.accel = 0
        self.targets.brake = 100
        self.relay.set(False)

    def _apply_reverse_policy(self, reverse_requested: bool) -> None:
        if reverse_requested:
            if self.auto_grant_reverse and not self.allow_reverse_manual:
                if not self.allow_reverse:
                    LOG.info("REVERSE req -> ALLOW_REVERSE=ON, relay=ON")
                self.allow_reverse = True
                self.relay.set(True)
            elif not self.auto_grant_reverse and not self.allow_reverse_manual:
                LOG.info("REVERSE req con auto_grant_reverse=OFF -> ignorado")
        else:
            if self.auto_grant_reverse and not self.allow_reverse_manual and self.allow_reverse:
                LOG.info("REVERSE sin pedido -> ALLOW_REVERSE=OFF, relay=OFF")
                self.allow_reverse = False
            if not self.allow_reverse_manual:
                self.relay.set(False)

    def _update_direction_change_block(self, now: float) -> bool:
        accel = self.targets.accel
        if accel > 0:
            sign = 1
        elif accel < 0:
            sign = -1
        else:
            sign = 0

        if sign == 0:
            self._direction_block_until = 0.0
            self._last_accel_sign = 0
            return False

        if self._last_accel_sign == 0:
            self._last_accel_sign = sign
            return False

        if sign != self._last_accel_sign:
            self._direction_block_until = now + self.settings.direction_change_delay_s
            self._last_accel_sign = sign
            LOG.warning(
                "Cambio de direccion -> esperando %.1fs sin acelerar",
                self.settings.direction_change_delay_s,
            )
            return True

        if self._direction_block_until:
            if now >= self._direction_block_until:
                LOG.info("Cambio de direccion habilitado")
                self._direction_block_until = 0.0
            else:
                return True

        return False

    def _apply_accel_ramp(self, target: int, now: float) -> int:
        if self._last_ramp_update is None:
            self._last_ramp_update = now
        dt = max(0.0, now - self._last_ramp_update)
        self._last_ramp_update = now

        current = self._ramp_value
        diff = float(target) - current
        if abs(diff) < 0.5 or self.settings.accel_ramp_time_s <= 0.0:
            self._ramp_value = float(target)
            return int(round(self._ramp_value))

        rate = abs(diff) / self.settings.accel_ramp_time_s
        step = rate * dt
        if step > abs(diff):
            step = abs(diff)
        self._ramp_value = current + (step if diff > 0 else -step)
        self._ramp_value = max(-100.0, min(100.0, self._ramp_value))
        return int(round(self._ramp_value))

    def prepare_command(self, now: float | None = None) -> CommandFrame:
        now = time.time() if now is None else now
        reverse_requested = self.status.reverse_req or (self.targets.accel < 0)
        self._apply_reverse_policy(reverse_requested)

        throttle_blocked = self._update_direction_change_block(now)
        desired_accel = clamp_int(self.targets.accel, -100, 100)
        send_accel = desired_accel
        send_brake = clamp_int(self.targets.brake, 0, 100)
        send_drive_enabled = self.drive_enabled

        if self.estop:
            send_accel = 0
            send_brake = 100
            send_drive_enabled = False
            self._ramp_value = 0.0
            self._last_ramp_update = now
        else:
            if throttle_blocked or desired_accel == 0:
                send_accel = 0
            if not send_drive_enabled:
                send_accel = 0
            if send_brake > 0:
                send_accel = 0
                self._ramp_value = 0.0
                self._last_ramp_update = now

        send_accel = self._apply_accel_ramp(send_accel, now)

        flags = CommandFlags(0)
        if self.estop:
            flags |= CommandFlags.ESTOP
        if send_drive_enabled:
            flags |= CommandFlags.DRIVE_EN
        if self.allow_reverse:
            flags |= CommandFlags.ALLOW_REVERSE

        command = CommandFrame(
            steer=clamp_int(self.targets.steer, -100, 100),
            accel=send_accel,
            brake=send_brake,
            flags=flags,
        )
        self.last_command = command
        self.last_command_ts = now
        return command

    def snapshot(self) -> dict:
        if self.status.last_packet_ts:
            age_ms = int((time.time() - self.status.last_packet_ts) * 1000)
        else:
            age_ms = None

        last_command = None
        if self.last_command:
            last_command = {
                "steer": int(self.last_command.steer),
                "accel": int(self.last_command.accel),
                "brake": int(self.last_command.brake),
                "flags": int(self.last_command.flags),
            }

        return {
            "drive_enabled": bool(self.drive_enabled),
            "allow_reverse": bool(self.allow_reverse),
            "relay": bool(self.relay.state),
            "auto_reverse": bool(self.auto_grant_reverse),
            "estop": bool(self.estop),
            "targets": {
                "steer": int(self.targets.steer),
                "accel": int(self.targets.accel),
                "brake": int(self.targets.brake),
            },
            "status": {
                "ready": bool(self.status.ready),
                "fault": bool(self.status.fault),
                "overcurrent": bool(self.status.overcurrent),
                "reverse_req": bool(self.status.reverse_req),
            },
            "telemetry": int(self.status.telemetry),
            "age_ms": age_ms,
            "packets_ok": int(self.status.packets_ok),
            "crc_errors": int(self.status.crc_errors),
            "last_command": last_command,
        }

    def describe(self) -> str:
        status_bits = []
        if self.status.ready:
            status_bits.append("READY")
        if self.status.fault:
            status_bits.append("FAULT")
        if self.status.overcurrent:
            status_bits.append("OVERCURRENT")
        if self.status.reverse_req:
            status_bits.append("REVERSE_REQ")
        if not status_bits:
            status_bits.append("NONE")

        if self.status.last_packet_ts:
            age_ms = (time.time() - self.status.last_packet_ts) * 1000
            age_desc = f"{age_ms:.0f}ms"
        else:
            age_desc = "N/A"

        return (
            f"Drive={'ON' if self.drive_enabled else 'OFF'}, "
            f"ReverseFlag={'ON' if self.allow_reverse else 'OFF'}, "
            f"Relay={'ON' if self.relay.state else 'OFF'}, "
            f"AutoReverse={'ON' if self.auto_grant_reverse else 'OFF'}, "
            f"E-Stop={'ON' if self.estop else 'OFF'}, "
            f"Targets(steer={self.targets.steer}, accel={self.targets.accel}, brake={self.targets.brake}), "
            f"ESP32(status={'+'.join(status_bits)}, telemetry={self.status.telemetry}, age={age_desc}, "
            f"rx_ok={self.status.packets_ok}, crc_err={self.status.crc_errors})"
        )
