from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    venv_python = "/home/docker/.venv/bin/python3"

    ws_to_joy = ExecuteProcess(
        cmd=[venv_python, "-m", "teleop.ws_to_joy"],
        output="screen",
    )

    joy_to_thrusters = Node(
        package="teleop",
        executable="joy_to_thrusters",
        output="screen",
    )

    return LaunchDescription([ws_to_joy, joy_to_thrusters])
