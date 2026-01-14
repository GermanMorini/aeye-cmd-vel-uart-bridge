# cmd_vel_uart_bridge

ROS 2 node that bridges `/cmd_vel_safe` to the UART protocol used by RASPY_SALUS.
The UART driver code is copied from RASPY_SALUS as-is (see `cmd_vel_uart_bridge/test_comms.py`).

## Usage

```bash
ros2 run cmd_vel_uart_bridge cmd_vel_uart_bridge
```

```bash
ros2 launch cmd_vel_uart_bridge cmd_vel_uart_bridge.launch.py
```

Web control (WebSocket -> UART, no ROS topic):

```bash
ros2 launch cmd_vel_uart_bridge control_uart_bridge.launch.py
```

The HTML UI is installed at `share/cmd_vel_uart_bridge/web/Control.html`.
Update the `RASPI_HOST` and `CAM_EMBED_URL` values inside the file to match your setup.

## Parameters (selected)

- `cmd_vel_topic` (string, default `/cmd_vel_safe`)
- `max_linear_speed` (float, default `2.77` m/s)
- `max_angular_speed` (float, default `1.7` rad/s)
- `deadband_linear` (float, default `0.02` m/s)
- `deadband_angular` (float, default `0.02` rad/s)
- `cmd_timeout` (float, default `0.5` s)
- `brake_on_stop_percent` (int, default `60`)
- `brake_on_timeout_percent` (int, default `100`)
- `dry_run` (bool, default `false`)

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
- `serial_port` (string, default empty)
- `baudrate` (int, default `0`)
- `serial_timeout` (float, default `-1.0`)
- `gpio_relay` (int, default `-1`)
- `status_timeout` (float, default `-1.0`)
- `gpio_off_list` (string, default empty)

## Dependencies

- `pyserial` is required to talk to the UART.
- `RPi.GPIO` is optional (only needed on Raspberry Pi for the relay).
- `websockets` is required for the WebSocket control server.
