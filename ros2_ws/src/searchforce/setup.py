import os
from glob import glob
from setuptools import setup

package_name = 'searchforce'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.functions'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
            ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moritz Hölper',
    maintainer_email='316987136+m-hoelper@users.noreply.github.com',
    description='Autonomous maze exploration, SLAM navigation and AprilTag object search (ROS 2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Laptop-side decision nodes: task1_*, task2_*. Robot-side: task3_object_detector (started by robot_bringup_launch.py)
            # Task 1: known maze, solved offline, driven cell by cell
            'task1_maze_requester = searchforce.task1_maze_requester:main',
            'task1_goal_sequencer = searchforce.task1_goal_sequencer:main',
            # Task 2: unknown maze, frontier exploration
            'task2_frontier_explorer = searchforce.task2_frontier_explorer:main',
            # Task 3: task 2 plus camera object search and maze reconstruction
            'task3_object_detector = searchforce.task3_object_detector:main',
            'task2_map_reconstruction = searchforce.task2_map_reconstruction:main',
        ],
    },
)
