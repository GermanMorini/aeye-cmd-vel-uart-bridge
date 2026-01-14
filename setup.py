from setuptools import setup

package_name = "cmd_vel_uart_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/bridge.launch.py",
                "launch/web_control.launch.py",
            ],
        ),
        ("share/" + package_name + "/web", ["web/Control.html"]),
    ],
    install_requires=["setuptools", "websockets"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="Bridge from /cmd_vel_safe to the UART protocol used by RASPY_SALUS.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_uart_bridge = cmd_vel_uart_bridge.cmd_vel_uart_bridge_node:main",
            "control_uart_bridge = cmd_vel_uart_bridge.control_uart_bridge:main",
        ],
    },
)
