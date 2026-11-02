
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.actions import Node
def generate_launch_description():
    controller_pkg = FindPackageShare('controller').find('controller')
    ldlidar_pkg = FindPackageShare('ldlidar_node').find('ldlidar_node')
    searchforce_pkg = FindPackageShare('searchforce').find('searchforce')
    orchestrator_launch = FindPackageShare('orchestrator_launch').find('orchestrator_launch')
    peripherals_pkg = FindPackageShare('peripherals').find('peripherals')

    params_file = os.path.join(FindPackageShare('searchforce').find('searchforce'), 'config', 'nav2_params_bringup_moritz.yaml')
    slam_file = os.path.join(FindPackageShare('searchforce').find('searchforce'), 'config', 'slam_moritz.yaml')
    
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, 'launch', 'controller.launch.py')
        )
    )

    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_pkg, 'launch', 'ldlidar.launch.py')
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orchestrator_launch, 'launch', 'slam_toolbox.launch.py')
            ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_file
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(searchforce_pkg, 'launch', 'navigation_launch.py')
            ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file
        }.items()
    )

    delay_slam_launch = TimerAction(
        period=10.0,     # delay in seconds
        actions=[slam_launch]
    )
    
    delay_nav2_launch = TimerAction(
        period=20.0,     # delay in seconds
        actions=[nav2_launch]
    )
    airtag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'apriltag.launch.py')
        )
    )

    pipline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'image_pipeline.launch.py')
        )
    )

    airtag_detector = Node(
        package='searchforce',
        executable='task3_object_detector',
        name='airtag_detector',
        output='screen'
    )

    delay_airtag_detector = TimerAction(
        period=10.0,     # delay in seconds
        actions=[airtag_detector]
    )

    return LaunchDescription([
        controller_launch,
        ldlidar_launch,
        delay_slam_launch,
        delay_nav2_launch,
        pipline_launch,
        airtag_launch,
        delay_airtag_detector
    ])

