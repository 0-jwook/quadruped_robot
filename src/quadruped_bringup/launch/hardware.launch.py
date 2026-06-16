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
            'body_height': 0.14,
            # SpotMicroAI BezierGait 파라미터 (통합 회전 운동학 + 고정 duty)
            'step_height': 0.035,   # swing 최대 발 들기 높이 (0.05→0.035, 다시 낮춤)
            'max_stride':  0.05,    # 발 stride 벡터 크기 상한 (속도 상한 결정)
            'period':      0.9,     # 전체 cycle Tstride (1.0→0.9, 속도 조금 ↑). max_speed = max_stride/(duty·period)
            'duty_trot':   0.6,     # trot stance 비율 (0.6 → 비행 구간 없음 + 속도 ↑)
            'duty_wave':   0.75,    # wave stance 비율 (3-leg 지지)
            'hip_x':       0.1225,  # 몸통중심~발 종방향 = BODY_L/2 (URDF 실측)
            'hip_y':       0.10,    # 몸통중심~발 횡방향 = BODY_W/2 + L1 = 0.07+0.03 (URDF 실측)
            'level_gain':  1.0,     # 수평 유지(중심 잡기) 강도 (0=끔, 1=완전 수평 유지 — 최대 30° 경사) ON
            'level_max':   0.09,    # 수평 유지 발 z 보정 상한 (m)
            'height_min':  0.07,    # 앉기 자세 가능 높이
            'height_max':  0.21,
            'gait_type':   'trot',  # 직진/회전=trot, 측방(게다리)=wave 자동 전환
            'cmd_vel_hold_time': 30.0,
            'pitch_offset': 0.015,  # rad. + = 앞 들기 (로봇 앞 기울임 보정)
            'roll_offset':  0.015,  # rad. + = 우측 들기 (로봇 우측 기울임 보정)
            'yaw_trim':     0.07,   # rad/s. 직진 휨 보정. 우측 휨 → 양수(좌향). period 0.9 기준 튜닝
            # 넘어짐 감지 + 자동 기립 (IMU roll/pitch 필요 — IMU 미수신이면 무동작)
            'fall_detect':  True,   # 넘어짐 감지 on/off
            'fall_tilt_thresh': 1.0,  # rad. 이 이상 기울면 넘어짐 (~57°)
            'auto_recover': False,  # 넘어지면 웅크렸다 밀어올려 자동 기립 (OFF). 켜려면 True
            'recover_time': 3.0,    # s. 기립 시퀀스 길이
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
