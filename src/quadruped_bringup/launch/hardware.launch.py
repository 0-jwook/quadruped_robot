"""
hardware.launch.py
==================
실제 STM32F103RB 하드웨어 연결 모드 런치 파일.

시뮬레이션 없이 gait_node + hardware_bridge 만 기동.
Gazebo / ros2_control / robot_state_publisher 는 시작하지 않음.

사용법:
  ros2 launch quadruped_bringup hardware.launch.py
  ros2 launch quadruped_bringup hardware.launch.py port:=/dev/ttyUSB0
  ros2 launch quadruped_bringup hardware.launch.py port:=/dev/ttyACM0 baudrate:=115200
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='STM32 시리얼 포트 (예: /dev/ttyACM0, /dev/ttyUSB0)',
    )
    baud_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='38400',
        description='UART 보드레이트',
    )

    gait_node = Node(
        package='quadruped_gait',
        executable='gait_node',
        name='gait_node',
        output='screen',
        parameters=[{
            'L1': 0.030,
            'L2': 0.115,
            'L3': 0.135,
            'body_height': 0.17,
            'step_height': 0.04,
            'max_stride':  0.05,
            'period':      0.8,
            'height_min':  0.11,
            'height_max':  0.21,
            'gait_type':   'trot',
        }],
    )

    hardware_bridge = Node(
        package='quadruped_gait',
        executable='hardware_bridge',
        name='hardware_bridge',
        output='screen',
        parameters=[{
            'port':     LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        gait_node,
        hardware_bridge,
    ])
