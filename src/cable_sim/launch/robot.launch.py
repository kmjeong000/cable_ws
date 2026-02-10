from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="bluerov2",
        description="Model namespace / name in Gazebo",
    )

    world = PathJoinSubstitution([
        FindPackageShare("cable_sim"),
        "world",
        "water_tank.world.sdf",
    ])

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v4", world],
        output="screen",
    )

    robot_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cable_sim"),
                "models",
                "bluerov2",
                "config",             
                "robot_config.py",
            ])
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
        }.items(),
    )

    return LaunchDescription([
        namespace_arg,
        gz_sim,
        robot_config,
    ])
