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
        default_value='115200',
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
            # SpotMicroAI BezierGait 파라미터 (통합 회전 운동학 + 고정 duty)
            'step_height': 0.035,   # swing 최대 발 들기 높이
            'max_stride':  0.05,    # 발 stride 벡터 크기 상한 (속도 상한 결정)
            'period':      0.5,     # 전체 cycle Tstride. max_speed = max_stride/(duty·period)
            'duty_trot':   0.6,     # trot stance 비율 (0.6 → 비행 구간 없음 + 속도 ↑)
            'duty_wave':   0.75,    # wave stance 비율 (3-leg 지지)
            'hip_x':       0.10,    # 몸통중심~발 종방향 (회전 운동학 — 실제 치수로 교체 권장)
            'hip_y':       0.05,    # 몸통중심~발 횡방향
            'height_min':  0.07,    # 앉기 자세 가능 높이
            'height_max':  0.21,
            'gait_type':   'trot',  # 직진/회전=trot, 측방(게다리)=wave 자동 전환
            'cmd_vel_hold_time': 30.0,
            'pitch_offset': 0.015,  # rad. + = 앞 들기 (로봇 앞 기울임 보정)
            'roll_offset':  0.015,  # rad. + = 우측 들기 (로봇 우측 기울임 보정)
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
