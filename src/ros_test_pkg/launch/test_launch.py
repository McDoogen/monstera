import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # ld.add_action(Node(
    #     package="ros_test_pkg",
    #     executable="my_node",
    #     name="derp"
    # ))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'test_bot.urdf'
    urdf = os.path.join(
        get_package_share_directory('ros_test_pkg'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]))
    
    ld.add_action(Node(
            package='ros_test_pkg',
            executable='my_node',
            name='derp',
            output='screen'))

    return ld