from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("max_speed_kmh", default_value="4.0"),
            DeclareLaunchArgument(
                "max_linear_speed",
                default_value=PythonExpression(
                    [LaunchConfiguration("max_speed_kmh"), " / 3.6"]
                ),
            ),
            DeclareLaunchArgument("max_accel_percent", default_value="29"),
            DeclareLaunchArgument("min_effective_speed", default_value="0.0"),
            DeclareLaunchArgument("linear_speed_offset", default_value="1.2"),
            DeclareLaunchArgument("max_angular_speed", default_value="1.7"),
            DeclareLaunchArgument("turning_radius", default_value="3.0"),
            DeclareLaunchArgument("max_steer_deg", default_value="30.0"),
            DeclareLaunchArgument("steer_mode", default_value="yaw_rate"),
            DeclareLaunchArgument("invert_steer", default_value="true"),
            DeclareLaunchArgument("invert_steer_in_reverse", default_value="true"),
            DeclareLaunchArgument("steer_limit", default_value="1.0"),
            DeclareLaunchArgument("deadband_linear", default_value="0.02"),
            DeclareLaunchArgument("deadband_angular", default_value="0.02"),
            DeclareLaunchArgument("cmd_timeout", default_value="10.0"),
            DeclareLaunchArgument("send_hz", default_value="20.0"),
            DeclareLaunchArgument("brake_on_stop_percent", default_value="60"),
            DeclareLaunchArgument("brake_on_timeout_percent", default_value="100"),
            DeclareLaunchArgument(
                "ws_url", default_value="ws://100.111.4.7:8765/controls"
            ),
            DeclareLaunchArgument("ws_reconnect_s", default_value="1.0"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("dry_run_log_interval", default_value="1.0"),
            DeclareLaunchArgument("tx_log_interval", default_value="0.2"),
            Node(
                package="cmd_vel_uart_bridge",
                executable="cmd_vel_ws_bridge",
                name="cmd_vel_ws_bridge",
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                        "max_accel_percent": LaunchConfiguration("max_accel_percent"),
                        "min_effective_speed": LaunchConfiguration(
                            "min_effective_speed"
                        ),
                        "linear_speed_offset": LaunchConfiguration(
                            "linear_speed_offset"
                        ),
                        "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                        "turning_radius": LaunchConfiguration("turning_radius"),
                        "max_steer_deg": LaunchConfiguration("max_steer_deg"),
                        "steer_mode": LaunchConfiguration("steer_mode"),
                        "invert_steer": LaunchConfiguration("invert_steer"),
                        "invert_steer_in_reverse": LaunchConfiguration(
                            "invert_steer_in_reverse"
                        ),
                        "steer_limit": LaunchConfiguration("steer_limit"),
                        "deadband_linear": LaunchConfiguration("deadband_linear"),
                        "deadband_angular": LaunchConfiguration("deadband_angular"),
                        "cmd_timeout": LaunchConfiguration("cmd_timeout"),
                        "send_hz": LaunchConfiguration("send_hz"),
                        "brake_on_stop_percent": LaunchConfiguration(
                            "brake_on_stop_percent"
                        ),
                        "brake_on_timeout_percent": LaunchConfiguration(
                            "brake_on_timeout_percent"
                        ),
                        "ws_url": LaunchConfiguration("ws_url"),
                        "ws_reconnect_s": LaunchConfiguration("ws_reconnect_s"),
                        "dry_run": LaunchConfiguration("dry_run"),
                        "dry_run_log_interval": LaunchConfiguration(
                            "dry_run_log_interval"
                        ),
                        "tx_log_interval": LaunchConfiguration("tx_log_interval"),
                    }
                ],
            ),
        ]
    )
