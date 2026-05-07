from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    pkg_dir = Path(__file__).resolve().parents[1]

    return LaunchDescription([
        Node(
            package="board_pose_ros",
            executable="board_pose_node",
            name="board_pose_node",
            output="screen",
            parameters=[{
                "image_topic": "/robot_09/oakd/rgb/image_raw/compressed",
                "camera_frame": "oak_camera_frame",
                "board_frame": "board_frame",
                "calibration_file": str(pkg_dir / "config" / "camera_calib_oak.npz"),
                "board_config_file": str(pkg_dir / "config" / "board_config.json"),
                "log_pose": True,
                "log_every_n": 10,
                "log_rpy_degrees": False,
            }]
        )
    ])
