from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='self_driving_sim',
            executable='simulator_node',
            name='simulator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])