from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('self_driving_sim')
rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='self_driving_sim',
            executable='simulator_node',
            name='simulator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='clothoid_motion_planner',
            executable='clot_motion_planner_node',
            name='clot_motion_planner_node',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])