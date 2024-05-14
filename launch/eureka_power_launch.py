from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_power_2',
            executable='battery_decoder',
            name='battery_decoder',
            shell=True,
        ),
    ])