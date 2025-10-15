from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rc_controller = Node(
        package='my_rc_car',
        executable='rc_controller',
        name='rc_control',
        output='screen',
        emulate_tty=True,
        
    )

    drive_control_canbus = Node(
        package='my_rc_car',
        executable='drive_control_canbus',
        name='canbus',
        output='screen',
        
    )

    return LaunchDescription([
        rc_controller,
        drive_control_canbus,
    ])
