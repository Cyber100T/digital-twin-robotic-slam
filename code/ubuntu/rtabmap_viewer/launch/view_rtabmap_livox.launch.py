from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    rviz_config = os.path.expanduser('~/rtabmap_viewer/config/rtabmap_livox_view.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        )
    ])
