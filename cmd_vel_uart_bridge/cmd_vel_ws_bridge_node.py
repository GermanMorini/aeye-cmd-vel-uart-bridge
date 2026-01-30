import asyncio
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

import websockets


class WsSender:
    def __init__(self, url: str, reconnect_s: float, logger):
        self._url = url
        self._reconnect_s = reconnect_s
        self._logger = logger
        self._loop = None
        self._queue = None
        self._thread = None
        self._stop_event = threading.Event()
        self._connected = False

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._queue = asyncio.Queue(maxsize=1)
        try:
            self._loop.run_until_complete(self._main())
        finally:
            self._loop.close()

    async def _main(self):
        while not self._stop_event.is_set():
            try:
                async with websockets.connect(self._url) as ws:
                    if not self._connected:
                        self._logger.info(f"WS connected: {self._url}")
                    self._connected = True
                    while not self._stop_event.is_set():
                        try:
                            data = await asyncio.wait_for(self._queue.get(), timeout=0.1)
                        except asyncio.TimeoutError:
                            continue
                        await ws.send(data)
            except Exception as exc:
                if self._connected:
                    self._logger.warn(f"WS disconnected: {exc}")
                else:
                    self._logger.warn(f"WS connect failed: {exc}")
                self._connected = False
                if self._stop_event.is_set():
                    break
                await asyncio.sleep(self._reconnect_s)
        self._connected = False

    def send(self, payload: dict):
        if not self._loop or not self._loop.is_running():
            return
        data = json.dumps(payload)

        def _enqueue():
            if self._queue is None:
                return
            try:
                while True:
                    self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            try:
                self._queue.put_nowait(data)
            except asyncio.QueueFull:
                pass

        self._loop.call_soon_threadsafe(_enqueue)

    def stop(self):
        self._stop_event.set()
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(lambda: None)
        if self._thread:
            self._thread.join(timeout=1.0)


class CmdVelWsBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_ws_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("max_linear_speed", 4.16)
        self.declare_parameter("min_effective_speed", 0.0)
        self.declare_parameter("linear_speed_offset", 1.5)
        self.declare_parameter("max_angular_speed", 2.5)
        self.declare_parameter("turning_radius", 1.7)
        self.declare_parameter("max_steer_deg", 30.0)
        self.declare_parameter("steer_mode", "yaw_rate")
        self.declare_parameter("invert_steer", True)
        self.declare_parameter("invert_steer_in_reverse", True)
        self.declare_parameter("steer_limit", 1.0)
        self.declare_parameter("deadband_linear", 0.02)
        self.declare_parameter("deadband_angular", 0.02)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("send_hz", 20.0)
        self.declare_parameter("brake_on_stop_percent", 60)
        self.declare_parameter("brake_on_timeout_percent", 100)
        self.declare_parameter("ws_url", "ws://100.111.4.7:8765/controls")
        self.declare_parameter("ws_reconnect_s", 1.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("dry_run_log_interval", 1.0)
        self.declare_parameter("tx_log_interval", 0.2)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.min_effective_speed = float(
            self.get_parameter("min_effective_speed").value
        )
        self.linear_speed_offset = float(
            self.get_parameter("linear_speed_offset").value
        )
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.turning_radius = float(self.get_parameter("turning_radius").value)
        self.max_steer_deg = float(self.get_parameter("max_steer_deg").value)
        self.steer_mode = str(self.get_parameter("steer_mode").value)
        self.invert_steer = bool(self.get_parameter("invert_steer").value)
        self.invert_steer_in_reverse = bool(
            self.get_parameter("invert_steer_in_reverse").value
        )
        self.steer_limit = float(self.get_parameter("steer_limit").value)
        self.deadband_linear = float(self.get_parameter("deadband_linear").value)
        self.deadband_angular = float(self.get_parameter("deadband_angular").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.send_hz = float(self.get_parameter("send_hz").value)
        self.brake_on_stop_percent = int(
            self.get_parameter("brake_on_stop_percent").value
        )
        self.brake_on_timeout_percent = int(
            self.get_parameter("brake_on_timeout_percent").value
        )
        self.ws_url = str(self.get_parameter("ws_url").value)
        self.ws_reconnect_s = float(self.get_parameter("ws_reconnect_s").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.dry_run_log_interval = float(
            self.get_parameter("dry_run_log_interval").value
        )
        self.tx_log_interval = float(self.get_parameter("tx_log_interval").value)

        self._normalize_params()

        self._sender = None
        if self.dry_run:
            self.get_logger().info("dry_run enabled: skipping WebSocket connection")
        else:
            self._sender = WsSender(self.ws_url, self.ws_reconnect_s, self.get_logger())
            self._sender.start()

        self._last_cmd = None
        self._last_cmd_time = None
        self._timed_out = True
        self._last_dry_log = 0.0
        self._last_tx_log = 0.0

        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_cb, 10)
        self._send_timer = self.create_timer(1.0 / self.send_hz, self._send_tick)

        self.get_logger().info(
            f"Forwarding {self.cmd_vel_topic} -> {self.ws_url} (dry_run={self.dry_run})"
        )

    def _normalize_params(self):
        if self.max_linear_speed <= 0.0:
            self.get_logger().warn("max_linear_speed <= 0, forcing 1.0")
            self.max_linear_speed = 1.0
        if self.min_effective_speed < 0.0:
            self.min_effective_speed = 0.0
        if self.min_effective_speed > self.max_linear_speed:
            self.get_logger().warn(
                "min_effective_speed > max_linear_speed, clamping to max_linear_speed"
            )
            self.min_effective_speed = self.max_linear_speed
        if self.linear_speed_offset < 0.0:
            self.linear_speed_offset = 0.0
        if self.linear_speed_offset > self.max_linear_speed:
            self.get_logger().warn(
                "linear_speed_offset > max_linear_speed, clamping to max_linear_speed"
            )
            self.linear_speed_offset = self.max_linear_speed
        if self.max_angular_speed <= 0.0:
            self.get_logger().warn("max_angular_speed <= 0, forcing 1.0")
            self.max_angular_speed = 1.0
        if self.cmd_timeout <= 0.0:
            self.get_logger().warn("cmd_timeout <= 0, forcing 0.5")
            self.cmd_timeout = 0.5
        if self.send_hz <= 0.0:
            self.get_logger().warn("send_hz <= 0, forcing 20")
            self.send_hz = 20.0
        if self.ws_reconnect_s <= 0.0:
            self.ws_reconnect_s = 1.0

        if self.deadband_linear < 0.0:
            self.deadband_linear = 0.0
        if self.deadband_angular < 0.0:
            self.deadband_angular = 0.0

        self.steer_mode = self.steer_mode.strip().lower()
        if self.steer_mode not in ("yaw_rate", "curvature", "steer_angle"):
            self.get_logger().warn(
                f"steer_mode '{self.steer_mode}' not recognized, using yaw_rate"
            )
            self.steer_mode = "yaw_rate"

        if self.turning_radius <= 0.0:
            self.get_logger().warn("turning_radius <= 0, disabling radius normalization")
            self.turning_radius = 0.0
        if self.max_steer_deg <= 0.0:
            self.get_logger().warn("max_steer_deg <= 0, disabling steer normalization")
            self.max_steer_deg = 0.0

        self._max_steer_rad = 0.0
        self._wheelbase = None
        if self.max_steer_deg >= 89.0:
            self.get_logger().warn("max_steer_deg too large, clamping to 89")
            self.max_steer_deg = 89.0

        if self.max_steer_deg > 0.0:
            self._max_steer_rad = math.radians(self.max_steer_deg)

        if self.turning_radius > 0.0 and self._max_steer_rad > 0.0:
            self._wheelbase = self.turning_radius * math.tan(self._max_steer_rad)
            self.max_angular_speed = self.max_linear_speed / self.turning_radius
            self.get_logger().info(
                "Using turning_radius="
                f"{self.turning_radius:.2f}m, max_steer_deg="
                f"{self.max_steer_deg:.1f} -> wheelbase="
                f"{self._wheelbase:.2f}m, max_angular_speed="
                f"{self.max_angular_speed:.3f}"
            )

        if self.invert_steer:
            self.get_logger().info("invert_steer enabled")

        if self.steer_limit < 0.0:
            self.steer_limit = 0.0
        if self.steer_limit > 1.0:
            self.get_logger().warn("steer_limit > 1.0, clamping to 1.0")
            self.steer_limit = 1.0

        self.brake_on_stop_percent = self._clamp(self.brake_on_stop_percent, 0, 100)
        self.brake_on_timeout_percent = self._clamp(
            self.brake_on_timeout_percent, 0, 100
        )
        if self.dry_run_log_interval < 0.0:
            self.dry_run_log_interval = 0.0
        if self.tx_log_interval < 0.0:
            self.tx_log_interval = 0.0

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()
        if self._timed_out:
            self.get_logger().info("cmd_vel recovered")
        self._timed_out = False

    def _send_tick(self):
        now = time.monotonic()
        timed_out = self._last_cmd_time is None or (
            now - self._last_cmd_time > self.cmd_timeout
        )

        if timed_out:
            if not self._timed_out:
                self.get_logger().warn("cmd_vel timeout, applying brake")
        else:
            if self._timed_out:
                self.get_logger().info("cmd_vel active")

        self._timed_out = timed_out

        linear_x = float(self._last_cmd.linear.x) if self._last_cmd else 0.0
        angular_z = float(self._last_cmd.angular.z) if self._last_cmd else 0.0

        throttle = self._normalize_linear(linear_x)
        accel_cmd = int(round(throttle * 100.0))
        steer_cmd = self._compute_steer_cmd(linear_x, angular_z)

        if timed_out:
            brake_cmd = self.brake_on_timeout_percent
            accel_cmd = 0
            steer_cmd = 0
            drive_enabled = False
        else:
            drive_enabled = True
            if (
                abs(linear_x) <= self.deadband_linear
                and abs(angular_z) <= self.deadband_angular
            ):
                brake_cmd = self.brake_on_stop_percent
                accel_cmd = 0
            else:
                brake_cmd = 0

        payload = {
            "throttle": self._clamp(accel_cmd, -100, 100) / 100.0,
            "steer": self._clamp(steer_cmd, -100, 100) / 100.0,
            "brake": brake_cmd > 0,
            "brake_percent": brake_cmd,
            "drive_enabled": drive_enabled,
        }

        if self.dry_run:
            self._maybe_log_dry_run(now, payload)
            return

        self._maybe_log_tx(now, payload)
        self._sender.send(payload)

    def _compute_steer_cmd(self, linear_x: float, angular_z: float) -> int:
        if abs(angular_z) <= self.deadband_angular:
            return 0

        if self.steer_mode == "steer_angle" and self._max_steer_rad > 0.0:
            steer_norm = angular_z / self._max_steer_rad
        elif (
            self.steer_mode == "curvature"
            and self._wheelbase
            and self._max_steer_rad > 0.0
        ):
            steer_angle = math.atan(self._wheelbase * angular_z)
            steer_norm = steer_angle / self._max_steer_rad
        elif self._wheelbase and self._max_steer_rad > 0.0:
            steer_angle = math.atan2(self._wheelbase * angular_z, linear_x)
            steer_norm = steer_angle / self._max_steer_rad
        else:
            steer_norm = self._normalize(
                angular_z, self.max_angular_speed, self.deadband_angular
            )

        if self.invert_steer:
            steer_norm = -steer_norm
        if self.invert_steer_in_reverse and linear_x < -self.deadband_linear:
            steer_norm = -steer_norm

        if self.steer_limit < 1.0:
            if steer_norm > self.steer_limit:
                steer_norm = self.steer_limit
            if steer_norm < -self.steer_limit:
                steer_norm = -self.steer_limit

        if steer_norm > 1.0:
            steer_norm = 1.0
        if steer_norm < -1.0:
            steer_norm = -1.0

        return int(round(steer_norm * 100.0))

    def _maybe_log_dry_run(self, now: float, payload: dict):
        if self.dry_run_log_interval <= 0.0:
            return
        if now - self._last_dry_log < self.dry_run_log_interval:
            return
        self._last_dry_log = now
        self.get_logger().info(f"dry_run payload={payload}")

    def _maybe_log_tx(self, now: float, payload: dict):
        if self.tx_log_interval <= 0.0:
            return
        if now - self._last_tx_log < self.tx_log_interval:
            return
        self._last_tx_log = now
        self.get_logger().info(f"tx payload={payload}")

    @staticmethod
    def _normalize(value: float, max_value: float, deadband: float) -> float:
        if abs(value) <= deadband:
            return 0.0
        if max_value <= 0.0:
            return 0.0
        scaled = value / max_value
        if scaled > 1.0:
            return 1.0
        if scaled < -1.0:
            return -1.0
        return float(scaled)

    def _normalize_linear(self, value: float) -> float:
        if abs(value) <= self.deadband_linear:
            return 0.0
        if self.max_linear_speed <= 0.0:
            return 0.0
        magnitude = abs(value)
        if self.linear_speed_offset > 0.0:
            magnitude += self.linear_speed_offset
        if self.min_effective_speed > 0.0 and magnitude < self.min_effective_speed:
            magnitude = self.min_effective_speed
        scaled = magnitude / self.max_linear_speed
        if scaled > 1.0:
            scaled = 1.0
        return float(math.copysign(scaled, value))

    @staticmethod
    def _clamp(value: int, lo: int, hi: int) -> int:
        if value < lo:
            return lo
        if value > hi:
            return hi
        return value

    def shutdown(self):
        if self._sender:
            self._sender.stop()


def main():
    rclpy.init()
    node = None
    try:
        node = CmdVelWsBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutdown requested (SIGINT)")
    except Exception as exc:
        if node:
            node.get_logger().error(f"Bridge failed: {exc}")
        else:
            print(f"Bridge failed: {exc}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
