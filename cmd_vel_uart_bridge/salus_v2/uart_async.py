"""Async UART transport using pyserial-asyncio."""
from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable

import serial_asyncio

from cmd_vel_uart_bridge.salus_v2.protocol import StatusFrame, StatusFrameParser

LOG = logging.getLogger(__name__)

StatusHandler = Callable[[StatusFrame], Awaitable[None] | None]


class UartLink:
    def __init__(
        self,
        port: str,
        baudrate: int,
        on_status: StatusHandler,
        on_crc_error: Callable[[int], None] | None = None,
    ):
        self._port = port
        self._baudrate = int(baudrate)
        self._on_status = on_status
        self._on_crc_error = on_crc_error
        self._parser = StatusFrameParser()
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._read_task: asyncio.Task | None = None
        self._running = False
        self._mock = False

    async def open(self) -> None:
        if self._port.startswith("loop://") or self._port.startswith("mock://"):
            self._running = True
            self._mock = True
            LOG.warning("UART en modo mock (%s); sin I/O real", self._port)
            return
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self._port, baudrate=self._baudrate
        )
        self._running = True
        self._read_task = asyncio.create_task(self._read_loop())
        LOG.info("UART abierto en %s @ %d", self._port, self._baudrate)

    async def close(self) -> None:
        self._running = False
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
        if self._writer:
            self._writer.close()
            wait_closed = getattr(self._writer, "wait_closed", None)
            if callable(wait_closed):
                await wait_closed()
        self._reader = None
        self._writer = None
        self._mock = False

    async def send(self, payload: bytes) -> None:
        if self._mock:
            return
        if not self._writer:
            return
        try:
            self._writer.write(payload)
            await self._writer.drain()
        except Exception as exc:
            LOG.error("UART write error: %s", exc)

    async def _read_loop(self) -> None:
        assert self._reader is not None
        while self._running:
            try:
                data = await self._reader.read(64)
            except Exception as exc:
                LOG.error("UART read error: %s", exc)
                await asyncio.sleep(0.05)
                continue
            if not data:
                await asyncio.sleep(0.01)
                continue
            frames, crc_errors = self._parser.feed(data)
            if crc_errors and self._on_crc_error:
                self._on_crc_error(crc_errors)
            for frame in frames:
                result = self._on_status(frame)
                if asyncio.iscoroutine(result):
                    await result
