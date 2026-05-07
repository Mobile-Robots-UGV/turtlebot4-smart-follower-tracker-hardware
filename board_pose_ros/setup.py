from setuptools import find_packages, setup
from glob import glob

package_name = "board_pose_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eva",
    maintainer_email="eva@example.com",
    description="Board pose estimation from compressed ROS image topic",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "board_pose_node = board_pose_ros.board_pose_node:main",
        ],
    },
)
