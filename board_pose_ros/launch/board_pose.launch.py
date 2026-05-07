from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_dir = Path(get_package_share_directory("board_pose_ros"))

    return LaunchDescription([
        Node(
            package="board_pose_ros",
            executable="board_pose_node",
            name="board_pose_node",
            output="screen",
            parameters=[{
                "image_topic": "/robot_09/oakd/rgb/image_raw/compressed",
                "image_type": "compressed",
                "camera_info_topic": "",
                "use_camera_info": False,
                "camera_frame": "oak_camera_frame",
                "board_frame": "board_frame",
                "calibration_file": str(pkg_dir / "config" / "camera_calib_oak.npz"),
                "board_config_file": str(pkg_dir / "config" / "board_config.json"),
                "publish_static_camera_tf": True,
                "publish_debug_image": True,
                "process_every_n": 1,
                "log_pose": True,
                "log_every_n": 10,
                "log_rpy_degrees": False,
            }]
        )
    ])
