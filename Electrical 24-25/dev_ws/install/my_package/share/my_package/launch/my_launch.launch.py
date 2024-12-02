from launch import LaunchDescription
from launch_ros.actions import Node
#  NO IDEA IF THIS WORKS EITHER
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            output='screen'
        )
    ])
