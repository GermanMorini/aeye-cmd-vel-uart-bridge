"""GPIO relay abstraction using gpiozero with fallback."""
from __future__ import annotations

import logging
from typing import Iterable

try:  # pragma: no cover - depends on hardware
    from gpiozero import Device, OutputDevice
    from gpiozero.pins.mock import MockFactory
except Exception:  # pragma: no cover - fallback
    Device = None
    OutputDevice = None
    MockFactory = None

LOG = logging.getLogger(__name__)


class Relay:
    def __init__(self, pin: int, off_pins: Iterable[int], use_mock: bool = False):
        self._pin = int(pin)
        self._relay = None
        self._extras = []
        self.state = False

        if OutputDevice is None:
            LOG.warning("gpiozero no disponible; relay en modo no-op")
            return

        if use_mock and MockFactory is not None:
            Device.pin_factory = MockFactory()

        off_set = {int(p) for p in off_pins}
        off_set.discard(self._pin)
        for pin_id in sorted(off_set):
            try:
                self._extras.append(OutputDevice(pin_id, initial_value=False))
            except Exception as exc:  # pragma: no cover
                LOG.warning("No se pudo inicializar GPIO %s: %s", pin_id, exc)

        try:
            self._relay = OutputDevice(self._pin, initial_value=False)
        except Exception as exc:  # pragma: no cover
            LOG.warning("No se pudo inicializar relay GPIO %s: %s", self._pin, exc)

    def set(self, on: bool) -> None:
        self.state = bool(on)
        if self._relay is None:
            return
        self._relay.value = 1 if self.state else 0

    def close(self) -> None:
        if self._relay is not None:
            self._relay.value = 0
            self._relay.close()
        for dev in self._extras:
            dev.value = 0
            dev.close()
