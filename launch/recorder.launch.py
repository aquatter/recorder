import launch
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


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
            name="camera_ros",
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
                "jpeg_quality:=50",
                "-p",
                "FrameDurationLimits:=[100000,100000]"
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            get_abs_path_to_launch_file("ros2_mpu6050", "launch/ros2_mpu6050.launch.py")
        )
    )

    ld.add_action(
        Node(
            package="ublox_gps",
            executable="ublox_gps_node",
            name="ublox_gps",
            ros_arguments=["-p", "device:=/dev/ttyAMA0"],
        )
    )

    ld.add_action(
            Node(
                package="image_viz",
                executable="ImageViz",
                name="image_viz"
            )
        )

    return ld
