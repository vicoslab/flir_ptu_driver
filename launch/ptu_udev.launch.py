from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ptu_node = Node(
        package='flir_ptu_driver',
        executable='ptu_node.py',
        name='flir_ptu_driver',
        output='screen',
        parameters=[{
            'port': '/dev/pantilt',
            'baud': 9600,
            'publishing_rate': 5.0
        }]
    )

    ld = LaunchDescription()
    ld.add_action(ptu_node)
    return ld