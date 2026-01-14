# AGENTS.md

Role
- You are a ROS 2 Humble expert with UART and Raspberry Pi GPIO knowledge.
- Prefer minimal, safe changes and keep the UART protocol identical to the copy from RASPY_SALUS.

Repo map (package)
- `cmd_vel_uart_bridge/cmd_vel_uart_bridge/cmd_vel_uart_bridge_node.py`: ROS 2 node bridging `/cmd_vel_safe` to UART.
- `cmd_vel_uart_bridge/cmd_vel_uart_bridge/test_comms.py`: UART driver copy from RASPY_SALUS (do not modify unless explicitly asked).
- `cmd_vel_uart_bridge/launch/cmd_vel_uart_bridge.launch.py`: launch entrypoint.
- `cmd_vel_uart_bridge/package.xml`, `cmd_vel_uart_bridge/setup.py`, `cmd_vel_uart_bridge/setup.cfg`.
- `cmd_vel_uart_bridge/README.md`: usage and parameters.

Behavior and safety
- Do not edit code in `RASPY_SALUS/`; only copy needed files into this package when requested.
- Keep CRC, flags, frame sizes, timing, and failsafe behavior consistent with `test_comms.py`.
- When adding parameters, wire them via `DeclareLaunchArgument` and document in `README.md`.
- Keep files ASCII only unless the file already uses non-ASCII.

ROS expectations
- Subscribe to `/cmd_vel_safe` (geometry_msgs/Twist), map to accel/steer in [-100, 100].
- Timeout => apply brake and disable drive.
- Stop (cmd_vel near zero) => apply brake_on_stop_percent and accel=0.

Testing
- Basic: `python3 -m py_compile cmd_vel_uart_bridge/cmd_vel_uart_bridge_node.py`.
- Dry run: `ros2 run cmd_vel_uart_bridge cmd_vel_uart_bridge --ros-args -p dry_run:=true`.
- Hardware: use `serial_port`/`baudrate` overrides and verify UART traffic.
