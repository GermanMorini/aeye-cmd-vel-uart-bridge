#!/usr/bin/env python3
"""
WebSocket -> UART bridge for Control.html.

- Listens on ws://0.0.0.0:8765/controls (adjust WS_HOST/PORT/PATH if needed).
- Receives JSON with throttle (-1..1), steer (-1..1), brake (bool), gear (1/2).
- Maps to CommsTester (test_comms.py) to send over UART.
- Cuts accel and disables drive if no command arrives in INACTIVITY_BRAKE_S.
"""
import asyncio
import json
import os
import threading
import time

import websockets  # pip install websockets

from cmd_vel_uart_bridge.test_comms import CommsTester


def _env_str(name: str, default: str) -> str:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


WS_HOST = _env_str("SALUS_WS_HOST", "0.0.0.0")
WS_PORT = _env_int("SALUS_WS_PORT", 8765)
WS_PATH = _env_str("SALUS_WS_PATH", "/controls")
INACTIVITY_BRAKE_S = _env_float("SALUS_WS_INACTIVITY_S", 0.4)
RESUME_MIN_INTERVAL_S = _env_float("SALUS_WS_RESUME_MIN_INTERVAL_S", 0.25)
RESUME_HITS_REQUIRED = _env_int("SALUS_WS_RESUME_HITS_REQUIRED", 2)

tester = CommsTester()
running = True
last_command_ts = 0.0
_tick_thread = None
_start_ts = time.time()
_command_count = 0
failsafe_active = False
_resume_hits = 0
paused_by_client = False


def _clamp(value, lo, hi, default=0.0):
    try:
        v = float(value)
    except (TypeError, ValueError):
        v = default
    return max(lo, min(hi, v))


def apply_web_command(payload):
    """
    Converts received JSON to CommsTester targets.
    """
    global last_command_ts, _command_count, failsafe_active, _resume_hits, paused_by_client

    if paused_by_client:
        # Ignore commands while the client is paused.
        return

    throttle = _clamp(payload.get("throttle"), -1.0, 1.0, 0.0)
    steer_norm = _clamp(payload.get("steer"), -1.0, 1.0, 0.0)
    brake = bool(payload.get("brake", False))

    try:
        gear = int(payload.get("gear", 2))
    except (TypeError, ValueError):
        gear = 2

    # Max torque: always use full accel range.
    accel_limit = 100
    accel_cmd = int(throttle * accel_limit)
    steer_cmd = int(steer_norm * 100)
    now = time.time()
    dt = (now - last_command_ts) if last_command_ts else None

    # If still in failsafe, only release when commands arrive fast enough.
    if failsafe_active:
        if dt is not None and dt < RESUME_MIN_INTERVAL_S:
            _resume_hits += 1
        else:
            _resume_hits = 0

        if _resume_hits >= RESUME_HITS_REQUIRED:
            failsafe_active = False
            _resume_hits = 0
            print("[WS] Failsafe released: continuous commands received")
        else:
            # Keep drive off (no brake) until conditions are met.
            tester.targets.accel = 0
            tester.targets.brake = 0
            tester.drive_enabled = False
            print("[WS] In failsafe (tab out of focus?). Waiting for commands...")
            last_command_ts = now
            _command_count += 1
            print(
                f"[WS] #{_command_count} +{(now - _start_ts)*1000:.0f}ms | "
                f"throttle={throttle:.2f} steer={steer_cmd}"
            )
            return

    if brake:
        tester.targets.accel = 0
        tester.targets.brake = 100
    else:
        tester.targets.accel = accel_cmd
        tester.targets.brake = 0

    tester.targets.steer = steer_cmd
    tester.drive_enabled = True  # each command re-enables drive

    _command_count += 1
    last_command_ts = now

    print(
        f"[WS] #{_command_count} +{(now - _start_ts)*1000:.0f}ms | "
        f"throttle={throttle:.2f} accel={tester.targets.accel} | "
        f"steer={tester.targets.steer} | brake={tester.targets.brake} | gear={gear}"
    )


def set_paused(state: bool):
    """
    Enable/disable pause requested by the client.
    In pause, no UART traffic is sent.
    """
    global paused_by_client, failsafe_active, last_command_ts, _resume_hits
    new_state = bool(state)
    if new_state == paused_by_client:
        return
    paused_by_client = new_state
    failsafe_active = False
    _resume_hits = 0
    last_command_ts = 0.0
    tester.targets.accel = 0
    tester.targets.brake = 0
    tester.drive_enabled = False
    if paused_by_client:
        print("[WS] Pause enabled by client -> no UART traffic until resume")
    else:
        print("[WS] Pause disabled -> UART resumes on new commands")


def tick_loop():
    """
    Keeps CommsTester.tick() running (100 Hz) and applies inactivity failsafe.
    """
    global running, failsafe_active, paused_by_client

    print("[WS-BRIDGE] Tick loop started")
    while running:
        if paused_by_client:
            time.sleep(0.01)
            continue

        now = time.time()
        if last_command_ts and (now - last_command_ts) > INACTIVITY_BRAKE_S:
            if not failsafe_active:
                print("[WS] Inactivity > INACTIVITY_BRAKE_S -> accel=0, drive OFF")
            tester.targets.accel = 0
            tester.targets.brake = 0
            tester.drive_enabled = False
            failsafe_active = True

        tester.tick()


def _get_ws_path(websocket):
    """
    Return handshake path depending on websockets version.
    ServerConnection (new API) does not have .path but exposes .request.path.
    """
    path = getattr(websocket, "path", None)
    if path:
        return path
    req = getattr(websocket, "request", None)
    if req:
        return getattr(req, "path", None) or getattr(req, "target", None)
    return None


async def handle_client(websocket):
    """
    Handle a WebSocket client (Control.html).
    """
    path = _get_ws_path(websocket)
    if path and path != WS_PATH:
        print(f"[WS-BRIDGE] Unexpected path: {path} (expected {WS_PATH}) -> closing")
        await websocket.close(code=4000, reason="Unsupported path")
        return
    elif not path:
        print("[WS-BRIDGE] Path unavailable (new websockets API); accepting client")

    addr = websocket.remote_address
    print(f"[WS-BRIDGE] Client connected: {addr}")

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                try:
                    message = message.decode("utf-8")
                except Exception:
                    continue

            try:
                payload = json.loads(message)
            except json.JSONDecodeError:
                print(f"[WS-BRIDGE] Message is not JSON: {message!r}")
                continue

            if not isinstance(payload, dict):
                print(f"[WS-BRIDGE] Unexpected payload: {payload!r}")
                continue

            if payload.get("type") == "pause":
                set_paused(bool(payload.get("value")))
                continue
            if "pause" in payload:
                set_paused(bool(payload.get("pause")))
                continue

            apply_web_command(payload)

    except websockets.ConnectionClosed:
        print(f"[WS-BRIDGE] Client disconnected: {addr}")
    except Exception as exc:
        print(f"[WS-BRIDGE] Handler error: {exc}")


async def main_async():
    global _tick_thread

    _tick_thread = threading.Thread(target=tick_loop, daemon=True)
    _tick_thread.start()

    async with websockets.serve(handle_client, WS_HOST, WS_PORT):
        print(f"[WS-BRIDGE] Listening on ws://{WS_HOST}:{WS_PORT}{WS_PATH}")
        await asyncio.Future()  # run until Ctrl+C


def main():
    global running

    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[WS-BRIDGE] Ctrl+C, shutting down...")
    finally:
        running = False
        try:
            if _tick_thread and _tick_thread.is_alive():
                _tick_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            tester.close()
        except Exception as exc:
            print("[WS-BRIDGE] Error closing tester:", exc)


if __name__ == "__main__":
    main()
