from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("max_linear_speed", default_value="4.16"),
            DeclareLaunchArgument("min_effective_speed", default_value="0.0"),
            DeclareLaunchArgument("linear_speed_offset", default_value="1.5"),
            DeclareLaunchArgument("max_angular_speed", default_value="1.7"),
            DeclareLaunchArgument("turning_radius", default_value="3.0"),
            DeclareLaunchArgument("max_steer_deg", default_value="30.0"),
            DeclareLaunchArgument("steer_mode", default_value="yaw_rate"),
            DeclareLaunchArgument("invert_steer", default_value="true"),
            DeclareLaunchArgument("deadband_linear", default_value="0.02"),
            DeclareLaunchArgument("deadband_angular", default_value="0.02"),
            DeclareLaunchArgument("cmd_timeout", default_value="0.5"),
            DeclareLaunchArgument("watchdog_hz", default_value="10.0"),
            DeclareLaunchArgument("brake_on_stop_percent", default_value="60"),
            DeclareLaunchArgument("brake_on_timeout_percent", default_value="100"),
            DeclareLaunchArgument("auto_grant_reverse", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("tx_log_interval", default_value="0.2"),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("baudrate", default_value="0"),
            DeclareLaunchArgument("serial_timeout", default_value="-1.0"),
            DeclareLaunchArgument("gpio_relay", default_value="-1"),
            DeclareLaunchArgument("status_timeout", default_value="-1.0"),
            DeclareLaunchArgument("gpio_off_list", default_value=""),
            DeclareLaunchArgument("gpio_mock", default_value="false"),
            DeclareLaunchArgument("cmd_tx_period", default_value="-1.0"),
            Node(
                package="cmd_vel_uart_bridge",
                executable="salus_bridge",
                name="salus_bridge",
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "max_linear_speed": LaunchConfiguration("max_linear_speed"),
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
                        "deadband_linear": LaunchConfiguration("deadband_linear"),
                        "deadband_angular": LaunchConfiguration("deadband_angular"),
                        "cmd_timeout": LaunchConfiguration("cmd_timeout"),
                        "watchdog_hz": LaunchConfiguration("watchdog_hz"),
                        "brake_on_stop_percent": LaunchConfiguration(
                            "brake_on_stop_percent"
                        ),
                        "brake_on_timeout_percent": LaunchConfiguration(
                            "brake_on_timeout_percent"
                        ),
                        "auto_grant_reverse": LaunchConfiguration("auto_grant_reverse"),
                        "dry_run": LaunchConfiguration("dry_run"),
                        "tx_log_interval": LaunchConfiguration("tx_log_interval"),
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "serial_timeout": LaunchConfiguration("serial_timeout"),
                        "gpio_relay": LaunchConfiguration("gpio_relay"),
                        "status_timeout": LaunchConfiguration("status_timeout"),
                        "gpio_off_list": LaunchConfiguration("gpio_off_list"),
                        "gpio_mock": LaunchConfiguration("gpio_mock"),
                        "cmd_tx_period": LaunchConfiguration("cmd_tx_period"),
                    }
                ],
            ),
        ]
    )
