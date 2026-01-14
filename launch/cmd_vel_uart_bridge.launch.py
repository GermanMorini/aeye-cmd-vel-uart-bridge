from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic", default_value="/cmd_vel_safe"
            ),
            DeclareLaunchArgument(
                "max_linear_speed", default_value="2.77"
            ),
            DeclareLaunchArgument(
                "max_angular_speed", default_value="1.7"
            ),
            DeclareLaunchArgument(
                "deadband_linear", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "deadband_angular", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "cmd_timeout", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "watchdog_hz", default_value="10.0"
            ),
            DeclareLaunchArgument(
                "brake_on_stop_percent", default_value="60"
            ),
            DeclareLaunchArgument(
                "brake_on_timeout_percent", default_value="100"
            ),
            DeclareLaunchArgument(
                "dry_run", default_value="false"
            ),
            DeclareLaunchArgument(
                "dry_run_log_interval", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "auto_grant_reverse", default_value="true"
            ),
            DeclareLaunchArgument(
                "serial_port", default_value=""
            ),
            DeclareLaunchArgument(
                "baudrate", default_value="0"
            ),
            DeclareLaunchArgument(
                "serial_timeout", default_value="-1.0"
            ),
            DeclareLaunchArgument(
                "gpio_relay", default_value="-1"
            ),
            DeclareLaunchArgument(
                "status_timeout", default_value="-1.0"
            ),
            DeclareLaunchArgument(
                "gpio_off_list", default_value=""
            ),
            Node(
                package="cmd_vel_uart_bridge",
                executable="cmd_vel_uart_bridge",
                name="cmd_vel_uart_bridge",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                        "max_angular_speed": LaunchConfiguration("max_angular_speed"),
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
                        "dry_run": LaunchConfiguration("dry_run"),
                        "dry_run_log_interval": LaunchConfiguration(
                            "dry_run_log_interval"
                        ),
                        "auto_grant_reverse": LaunchConfiguration(
                            "auto_grant_reverse"
                        ),
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "serial_timeout": LaunchConfiguration("serial_timeout"),
                        "gpio_relay": LaunchConfiguration("gpio_relay"),
                        "status_timeout": LaunchConfiguration("status_timeout"),
                        "gpio_off_list": LaunchConfiguration("gpio_off_list"),
                    }
                ],
            ),
        ]
    )
