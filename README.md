# cmd_vel_uart_bridge

ROS 2 node that bridges `/cmd_vel_safe` to the UART protocol used by RASPY_SALUS.
The UART driver code is copied from RASPY_SALUS as-is (see `cmd_vel_uart_bridge/test_comms.py`).

## Usage

```bash
ros2 run cmd_vel_uart_bridge cmd_vel_uart_bridge
```

```bash
ros2 launch cmd_vel_uart_bridge bridge.launch.py
```

```bash
ros2 launch cmd_vel_uart_bridge bridge.launch.py max_speed_kmh:=4.0
```

```bash
ros2 launch cmd_vel_uart_bridge bridge.launch.py max_accel_percent:=29
```

Salus v2 bridge (optional alternative controller):

```bash
ros2 launch cmd_vel_uart_bridge salus_bridge.launch.py
```

Web control (WebSocket -> UART, no ROS topic):

```bash
ros2 launch cmd_vel_uart_bridge web_control.launch.py
```

The launch also serves the HTML UI over HTTP (default `http://0.0.0.0:8000/`).
The HTML file is installed at `share/cmd_vel_uart_bridge/web/Control.html`.
By default it uses the page hostname for the WebSocket and camera URLs.
Adjust `CAM_EMBED_URL` if your camera is hosted elsewhere.

WebSocket cmd_vel forwarder (ROS -> WebSocket):

```bash
ros2 launch cmd_vel_uart_bridge ws_bridge.launch.py
```

This connects to the WebSocket control server (from `web_control.launch.py`) and
forwards `/cmd_vel_safe` after scaling and deadband.

## Parameters (selected)

- `cmd_vel_topic` (string, default `/cmd_vel_safe`)
- `max_speed_kmh` (float, default `4.0`, converted internally to m/s)
- `max_linear_speed` (float, default `1.11` m/s, ~4 km/h)
- `max_accel_percent` (int, default `29`, hard cap for outgoing accel command)
- `min_effective_speed` (float, default `0.0` m/s, minimum non-zero command)
- `linear_speed_offset` (float, default `1.2` m/s, added to non-zero commands)
- `max_angular_speed` (float, default `1.7` rad/s)
- `turning_radius` (float, default `3.0` m, overrides `max_angular_speed` when > 0)
- `max_steer_deg` (float, default `30.0` deg, max steering angle)
- `steer_mode` (string, default `yaw_rate`: `yaw_rate`, `curvature`, or `steer_angle`)
- `invert_steer` (bool, default `true`)
- `invert_steer_in_reverse` (bool, default `true`)
- `steer_limit` (float, default `1.0`, clamps steer magnitude)
- `deadband_linear` (float, default `0.02` m/s)
- `deadband_angular` (float, default `0.02` rad/s)
- `cmd_timeout` (float, default `10.0` s)
- `brake_on_stop_percent` (int, default `60`)
- `brake_on_timeout_percent` (int, default `100`)
- `dry_run` (bool, default `false`)
- `tx_log_interval` (float, default `0.2` s, logs TX targets when > 0)

Serial overrides (optional, use to set env vars for the driver):
- `serial_port` (string, default empty)
- `baudrate` (int, default 0)
- `serial_timeout` (float, default -1.0)
- `gpio_relay` (int, default -1)
- `status_timeout` (float, default -1.0)
- `gpio_off_list` (string, default empty)

Web control launch arguments:

- `ws_host` (string, default `0.0.0.0`)
- `ws_port` (int, default `8765`)
- `ws_path` (string, default `/controls`)
- `ws_inactivity_brake_s` (float, default `0.4`)
- `ws_resume_min_interval_s` (float, default `0.25`)
- `ws_resume_hits_required` (int, default `2`)
- `max_accel_percent` (int, default `29`, hard cap for outgoing accel command)
- `http_host` (string, default `0.0.0.0`)
- `http_port` (int, default `8000`)
- `http_root` (string, default empty, auto-resolve)
- `serial_port` (string, default empty)
- `baudrate` (int, default `0`)
- `serial_timeout` (float, default `-1.0`)
- `gpio_relay` (int, default `-1`)
- `status_timeout` (float, default `-1.0`)
- `gpio_off_list` (string, default empty)

WebSocket cmd_vel forwarder launch arguments:

- `ws_url` (string, default `ws://100.111.4.7:8765/controls`)
- `ws_reconnect_s` (float, default `1.0`)
- `send_hz` (float, default `20.0`)
- `cmd_vel_topic` (string, default `/cmd_vel_safe`)
- `max_speed_kmh` (float, default `4.0`, converted internally to m/s)
- `max_linear_speed` (float, default `1.11` m/s, ~4 km/h)
- `max_accel_percent` (int, default `29`, hard cap for outgoing accel command)
- `min_effective_speed` (float, default `0.0` m/s, minimum non-zero command)
- `linear_speed_offset` (float, default `1.2` m/s, added to non-zero commands)
- `max_angular_speed` (float, default `1.7` rad/s)
- `turning_radius` (float, default `3.0` m, overrides `max_angular_speed` when > 0)
- `max_steer_deg` (float, default `30.0` deg, max steering angle)
- `steer_mode` (string, default `yaw_rate`: `yaw_rate`, `curvature`, or `steer_angle`)
- `invert_steer` (bool, default `true`)
- `invert_steer_in_reverse` (bool, default `true`)
- `deadband_linear` (float, default `0.02` m/s)
- `deadband_angular` (float, default `0.02` rad/s)
- `cmd_timeout` (float, default `0.5` s)
- `brake_on_stop_percent` (int, default `60`)
- `brake_on_timeout_percent` (int, default `100`)
- `dry_run` (bool, default `false`)
- `dry_run_log_interval` (float, default `1.0`)
- `tx_log_interval` (float, default `0.2` s, logs TX payload when > 0)

Salus v2 bridge launch arguments (optional alternative):

- `cmd_vel_topic` (string, default `/cmd_vel_safe`)
- `max_speed_kmh` (float, default `4.0`, converted internally to m/s)
- `max_linear_speed` (float, default `1.11` m/s, ~4 km/h)
- `max_accel_percent` (int, default `29`, hard cap for outgoing accel command)
- `min_effective_speed` (float, default `0.0` m/s, minimum non-zero command)
- `linear_speed_offset` (float, default `1.2` m/s, added to non-zero commands)
- `max_angular_speed` (float, default `1.7` rad/s)
- `turning_radius` (float, default `3.0` m, overrides `max_angular_speed` when > 0)
- `max_steer_deg` (float, default `30.0` deg, max steering angle)
- `steer_mode` (string, default `yaw_rate`: `yaw_rate`, `curvature`, or `steer_angle`)
- `invert_steer` (bool, default `true`)
- `invert_steer_in_reverse` (bool, default `true`)
- `deadband_linear` (float, default `0.02` m/s)
- `deadband_angular` (float, default `0.02` rad/s)
- `cmd_timeout` (float, default `0.5` s)
- `watchdog_hz` (float, default `10.0`)
- `brake_on_stop_percent` (int, default `60`)
- `brake_on_timeout_percent` (int, default `100`)
- `auto_grant_reverse` (bool, default `true`)
- `dry_run` (bool, default `false`, uses mock UART + GPIO)
- `tx_log_interval` (float, default `0.2` s, logs TX frames when > 0)
- `serial_port` (string, default empty)
- `baudrate` (int, default `0`)
- `serial_timeout` (float, default `-1.0`)
- `gpio_relay` (int, default `-1`)
- `status_timeout` (float, default `-1.0`)
- `gpio_off_list` (string, default empty)
- `gpio_mock` (bool, default `false`)
- `cmd_tx_period` (float, default `-1.0`)

## Dependencies

- `pyserial` is required to talk to the UART.
- `RPi.GPIO` is optional (only needed on Raspberry Pi for the relay).
- `websockets` is required for the WebSocket control server and client.
- Salus v2 bridge additionally uses `pyserial-asyncio`, `gpiozero`, `crccheck`,
  `pydantic`, and `pydantic-settings`.
