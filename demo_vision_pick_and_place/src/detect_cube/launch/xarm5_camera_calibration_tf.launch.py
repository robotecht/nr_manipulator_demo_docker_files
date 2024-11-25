""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: link_tcp -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "link_tcp",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "0.119399",
                "--y",
                "-0.140372",
                "--z",
                "-1.19059",
                "--qx",
                "0.0004347",
                "--qy",
                "0.0049228",
                "--qz",
                "0.714762",
                "--qw",
                "0.69935",
                # "--roll",
                # "3.13516",
                # "--pitch",
                # "3.13409",
                # "--yaw",
                # "-1.54898",
            ],
        ),
    ]
    return LaunchDescription(nodes)
