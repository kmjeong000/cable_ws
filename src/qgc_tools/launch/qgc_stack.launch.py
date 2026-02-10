from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/model/bluerov2/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/model/bluerov2/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
    )

    telemetry = ExecuteProcess(
        cmd=[
            "/home/docker/.venv/bin/python3",
            "/home/docker/cable_ws/src/qgc_tools/qgc_tools/qgc_telemetry.py",
        ],
        output="screen"
    )

    video = Node(
        package="qgc_tools",
        executable="qgc_image",
        output="screen"
    )

    return LaunchDescription([bridge, telemetry, video])