from setuptools import setup
from glob import glob
import os

package_name = 'sft_hardware_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'materials', 'textures'),
            glob('materials/textures/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eva',
    maintainer_email='eva@example.com',
    description='Hardware ArUco board tracker and recovery follower for TurtleBot 4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board_tracker_node = sft_hardware_tracker.board_tracker_node:main',
            'leader_odom_tf_node = sft_hardware_tracker.leader_odom_tf_node:main',
            'recovery_follower_node = sft_hardware_tracker.recovery_follower_node:main',
        ],
    },
)
