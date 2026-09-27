from launch import LaunchDescription
from launch_ros.actions import Node
import os



def generate_launch_description():


    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(os.getcwd(), "src/path_planner", 'rviz', 'map.rviz')]]
        ),

        Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            name='baselink_broadcaster',
            arguments = ["1", "0", "0", "0", "0", "1", "map", "base_link"]),

    ])