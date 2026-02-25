import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def _set_env(context, *args, **kwargs):
    actions = []

    def add_env(name, value):
        if value is None or value == "":
            return
        actions.append(SetEnvironmentVariable(name, value))

    add_env("SALUS_WS_HOST", LaunchConfiguration("ws_host").perform(context))
    add_env("SALUS_WS_PORT", LaunchConfiguration("ws_port").perform(context))
    add_env("SALUS_WS_PATH", LaunchConfiguration("ws_path").perform(context))
    add_env(
        "SALUS_WS_INACTIVITY_S",
        LaunchConfiguration("ws_inactivity_brake_s").perform(context),
    )
    add_env(
        "SALUS_WS_RESUME_MIN_INTERVAL_S",
        LaunchConfiguration("ws_resume_min_interval_s").perform(context),
    )
    add_env(
        "SALUS_WS_RESUME_HITS_REQUIRED",
        LaunchConfiguration("ws_resume_hits_required").perform(context),
    )
    add_env("SALUS_HTTP_HOST", LaunchConfiguration("http_host").perform(context))
    add_env("SALUS_HTTP_PORT", LaunchConfiguration("http_port").perform(context))

    http_root = LaunchConfiguration("http_root").perform(context)
    if http_root:
        add_env("SALUS_HTTP_ROOT", http_root)

    serial_port = LaunchConfiguration("serial_port").perform(context)
    if serial_port:
        add_env("SALUS_SERIAL_PORT", serial_port)

    baudrate_raw = LaunchConfiguration("baudrate").perform(context)
    try:
        baudrate = int(baudrate_raw)
    except (TypeError, ValueError):
        baudrate = 0
    if baudrate > 0:
        add_env("SALUS_BAUDRATE", str(baudrate))

    serial_timeout_raw = LaunchConfiguration("serial_timeout").perform(context)
    try:
        serial_timeout = float(serial_timeout_raw)
    except (TypeError, ValueError):
        serial_timeout = -1.0
    if serial_timeout >= 0.0:
        add_env("SALUS_TIMEOUT_S", str(serial_timeout))

    gpio_relay_raw = LaunchConfiguration("gpio_relay").perform(context)
    try:
        gpio_relay = int(gpio_relay_raw)
    except (TypeError, ValueError):
        gpio_relay = -1
    if gpio_relay >= 0:
        add_env("SALUS_GPIO_RELAY", str(gpio_relay))

    status_timeout_raw = LaunchConfiguration("status_timeout").perform(context)
    try:
        status_timeout = float(status_timeout_raw)
    except (TypeError, ValueError):
        status_timeout = -1.0
    if status_timeout >= 0.0:
        add_env("SALUS_STATUS_TIMEOUT_S", str(status_timeout))

    gpio_off_list = LaunchConfiguration("gpio_off_list").perform(context)
    if gpio_off_list:
        add_env("SALUS_GPIO_OFF_LIST", gpio_off_list)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ws_port", default_value="8765"),
            DeclareLaunchArgument("ws_path", default_value="/controls"),
            DeclareLaunchArgument("ws_inactivity_brake_s", default_value="0.4"),
            DeclareLaunchArgument("ws_resume_min_interval_s", default_value="0.25"),
            DeclareLaunchArgument("ws_resume_hits_required", default_value="2"),
            DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("http_port", default_value="8000"),
            DeclareLaunchArgument("http_root", default_value=""),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("baudrate", default_value="0"),
            DeclareLaunchArgument("serial_timeout", default_value="-1.0"),
            DeclareLaunchArgument("gpio_relay", default_value="-1"),
            DeclareLaunchArgument("status_timeout", default_value="-1.0"),
            DeclareLaunchArgument("gpio_off_list", default_value=""),
            OpaqueFunction(function=_set_env),
            ExecuteProcess(
                cmd=[sys.executable, "-m", "cmd_vel_uart_bridge.control_uart_bridge"],
                output="screen",
                emulate_tty=True,
                additional_env={"PYTHONUNBUFFERED": "1"},
            ),
        ]
    )
