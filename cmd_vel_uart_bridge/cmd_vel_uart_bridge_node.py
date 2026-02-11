import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from cmd_vel_uart_bridge import test_comms


class DryRunDriver:
    def __init__(self):
        self.targets = test_comms.TargetState()
        self.drive_enabled = False
        self.estop = False
        self.allow_reverse = False
        self.auto_grant_reverse = False

    def start_background_loop(self):
        return

    def close(self):
        return


class CmdVelUartBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_uart_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("max_linear_speed", 1.11)
        self.declare_parameter("max_accel_percent", 29)
        self.declare_parameter("min_effective_speed", 0.0)
        self.declare_parameter("linear_speed_offset", 1.2)
        self.declare_parameter("max_angular_speed", 2.5)
        self.declare_parameter("turning_radius", 1.7)
        self.declare_parameter("max_steer_deg", 30.0)
        self.declare_parameter("steer_mode", "yaw_rate")
        self.declare_parameter("invert_steer", True)
        self.declare_parameter("invert_steer_in_reverse", True)
        self.declare_parameter("deadband_linear", 0.02)
        self.declare_parameter("deadband_angular", 0.02)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("watchdog_hz", 10.0)
        self.declare_parameter("brake_on_stop_percent", 60)
        self.declare_parameter("brake_on_timeout_percent", 100)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("dry_run_log_interval", 1.0)
        self.declare_parameter("tx_log_interval", 0.2)
        self.declare_parameter("auto_grant_reverse", True)

        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", 0)
        self.declare_parameter("serial_timeout", -1.0)
        self.declare_parameter("gpio_relay", -1)
        self.declare_parameter("status_timeout", -1.0)
        self.declare_parameter("gpio_off_list", "")

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_accel_percent = int(self.get_parameter("max_accel_percent").value)
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
        self.deadband_linear = float(self.get_parameter("deadband_linear").value)
        self.deadband_angular = float(self.get_parameter("deadband_angular").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.watchdog_hz = float(self.get_parameter("watchdog_hz").value)
        self.brake_on_stop_percent = int(
            self.get_parameter("brake_on_stop_percent").value
        )
        self.brake_on_timeout_percent = int(
            self.get_parameter("brake_on_timeout_percent").value
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.dry_run_log_interval = float(
            self.get_parameter("dry_run_log_interval").value
        )
        self.tx_log_interval = float(self.get_parameter("tx_log_interval").value)
        self.auto_grant_reverse = bool(self.get_parameter("auto_grant_reverse").value)

        self._apply_serial_overrides()
        self._normalize_params()

        self._driver = self._create_driver()
        self._driver.drive_enabled = False
        self._driver.estop = False

        self._last_cmd_time = None
        self._timed_out = True
        self._last_dry_log = 0.0
        self._last_tx_log = 0.0

        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_cb, 10)
        self._watchdog_timer = self.create_timer(
            1.0 / self.watchdog_hz, self._watchdog_tick
        )

        self._apply_timeout()
        self._start_driver_loop()

        self.get_logger().info(
            f"Bridging {self.cmd_vel_topic} -> UART (dry_run={self.dry_run})"
        )

    def _apply_serial_overrides(self):
        serial_port = self.get_parameter("serial_port").value
        if serial_port:
            os.environ["SALUS_SERIAL_PORT"] = serial_port

        baudrate = int(self.get_parameter("baudrate").value)
        if baudrate > 0:
            os.environ["SALUS_BAUDRATE"] = str(baudrate)

        serial_timeout = float(self.get_parameter("serial_timeout").value)
        if serial_timeout >= 0.0:
            os.environ["SALUS_TIMEOUT_S"] = str(serial_timeout)

        gpio_relay = int(self.get_parameter("gpio_relay").value)
        if gpio_relay >= 0:
            os.environ["SALUS_GPIO_RELAY"] = str(gpio_relay)

        status_timeout = float(self.get_parameter("status_timeout").value)
        if status_timeout >= 0.0:
            os.environ["SALUS_STATUS_TIMEOUT_S"] = str(status_timeout)

        gpio_off_list = self.get_parameter("gpio_off_list").value
        if gpio_off_list:
            os.environ["SALUS_GPIO_OFF_LIST"] = gpio_off_list

    def _normalize_params(self):
        if self.max_linear_speed <= 0.0:
            self.get_logger().warn("max_linear_speed <= 0, forcing 1.0")
            self.max_linear_speed = 1.0
        if self.max_accel_percent < 0:
            self.get_logger().warn("max_accel_percent < 0, forcing 0")
            self.max_accel_percent = 0
        if self.max_accel_percent > 100:
            self.get_logger().warn("max_accel_percent > 100, clamping to 100")
            self.max_accel_percent = 100
        if self.min_effective_speed < 0.0:
            self.min_effective_speed = 0.0
        if self.min_effective_speed > self.max_linear_speed:
            self.get_logger().warn(
                "min_effective_speed > max_linear_speed, clamping to max_linear_speed"
            )
            self.min_effective_speed = self.max_linear_speed
        if self.linear_speed_offset < 0.0:
            self.linear_speed_offset = 0.0
        if self.max_angular_speed <= 0.0:
            self.get_logger().warn("max_angular_speed <= 0, forcing 1.0")
            self.max_angular_speed = 1.0
        if self.cmd_timeout <= 0.0:
            self.get_logger().warn("cmd_timeout <= 0, forcing 0.5")
            self.cmd_timeout = 0.5
        if self.watchdog_hz <= 0.0:
            self.get_logger().warn("watchdog_hz <= 0, forcing 10")
            self.watchdog_hz = 10.0

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

        self.brake_on_stop_percent = self._clamp(self.brake_on_stop_percent, 0, 100)
        self.brake_on_timeout_percent = self._clamp(
            self.brake_on_timeout_percent, 0, 100
        )
        if self.dry_run_log_interval < 0.0:
            self.dry_run_log_interval = 0.0
        if self.tx_log_interval < 0.0:
            self.tx_log_interval = 0.0

    def _create_driver(self):
        if self.dry_run:
            return DryRunDriver()

        try:
            driver = test_comms.CommsTester()
        except Exception as exc:
            raise RuntimeError(f"Failed to start UART driver: {exc}") from exc

        if hasattr(driver, "auto_grant_reverse"):
            driver.auto_grant_reverse = self.auto_grant_reverse

        return driver

    def _start_driver_loop(self):
        if self.dry_run:
            return
        if hasattr(self._driver, "start_background_loop"):
            self._driver.start_background_loop()

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = time.monotonic()
        self._apply_command(msg)
        if self._timed_out:
            self.get_logger().info("cmd_vel recovered")
        self._timed_out = False

    def _apply_command(self, msg: Twist):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        throttle = self._normalize_linear(linear_x)
        accel_cmd = int(round(throttle * 100.0))
        accel_cmd = self._clamp(
            accel_cmd, -self.max_accel_percent, self.max_accel_percent
        )
        steer_cmd = self._compute_steer_cmd(linear_x, angular_z)

        if abs(linear_x) <= self.deadband_linear and abs(angular_z) <= self.deadband_angular:
            brake_cmd = self.brake_on_stop_percent
            accel_cmd = 0
        else:
            brake_cmd = 0

        self._set_targets(steer_cmd, accel_cmd, brake_cmd, drive_enabled=True)

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

        if steer_norm > 1.0:
            steer_norm = 1.0
        if steer_norm < -1.0:
            steer_norm = -1.0

        return int(round(steer_norm * 100.0))

    def _apply_timeout(self):
        self._set_targets(
            steer=0,
            accel=0,
            brake=self.brake_on_timeout_percent,
            drive_enabled=False,
        )

    def _watchdog_tick(self):
        now = time.monotonic()
        timed_out = self._last_cmd_time is None or (
            now - self._last_cmd_time > self.cmd_timeout
        )

        if timed_out:
            if not self._timed_out:
                self.get_logger().warn("cmd_vel timeout, applying brake")
            self._apply_timeout()
        else:
            if self._timed_out:
                self.get_logger().info("cmd_vel active")

        self._timed_out = timed_out
        self._maybe_log_dry_run(now)
        self._maybe_log_tx(now)

    def _maybe_log_dry_run(self, now: float):
        if not self.dry_run or self.dry_run_log_interval <= 0.0:
            return
        if now - self._last_dry_log < self.dry_run_log_interval:
            return
        self._last_dry_log = now
        self.get_logger().info(
            "dry_run targets="
            f"steer={self._driver.targets.steer} "
            f"accel={self._driver.targets.accel} "
            f"brake={self._driver.targets.brake} "
            f"drive={'on' if self._driver.drive_enabled else 'off'}"
        )

    def _maybe_log_tx(self, now: float):
        if self.dry_run or self.tx_log_interval <= 0.0:
            return
        if now - self._last_tx_log < self.tx_log_interval:
            return
        self._last_tx_log = now
        self.get_logger().info(
            "tx targets="
            f"steer={self._driver.targets.steer} "
            f"accel={self._driver.targets.accel} "
            f"brake={self._driver.targets.brake} "
            f"drive={'on' if self._driver.drive_enabled else 'off'}"
        )

    def _set_targets(self, steer: int, accel: int, brake: int, drive_enabled: bool):
        self._driver.targets.steer = self._clamp(steer, -100, 100)
        self._driver.targets.accel = self._clamp(accel, -100, 100)
        self._driver.targets.brake = self._clamp(brake, 0, 100)
        self._driver.drive_enabled = drive_enabled

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
        if self._driver:
            try:
                self._driver.close()
            except Exception as exc:
                self.get_logger().warn(f"Error closing driver: {exc}")


def main():
    rclpy.init()
    node = None
    try:
        node = CmdVelUartBridge()
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
