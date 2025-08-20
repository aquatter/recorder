import launch_ros
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription


def get_abs_path_to_launch_file(package_name, rel_path):
    path_to_package = launch_ros.substitutions.FindPackageShare(package_name)
    abs_path_to_launch = launch.substitutions.PathJoinSubstitution(
        [path_to_package, rel_path]
    )
    return launch.actions.include_launch_description.AnyLaunchDescriptionSource(
        abs_path_to_launch
    )


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            ros_arguments=[
                "-p",
                "camera:=0",
                "-p",
                "width:=3840",
                "-p",
                "height:=2400",
                "-p",
                "format:=BGR888",
                "-p",
                "jpeg_quality:=70",
                "-p",
                "FrameDurationLimits:=[100000,100000]",
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            get_abs_path_to_launch_file("ros2_mpu6050", "launch/ros2_mpu6050.launch.py")
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            get_abs_path_to_launch_file("gpsd_client", "launch/gpsd_client-launch.py")
        )
    )

    ld.add_action(Node(package="image_viz", executable="ImageViz", name="image_viz"))

    ld.add_action(
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--start-paused",
                "--compression-mode",
                "file",
                "-d",
                "30",
                "--topics",
                "/camera/image_raw/compressed",
                "/imu/mpu6050",
                "/fix",
            ],
            output="screen",
            cwd="/root/data",
        )
    )

    return ld
