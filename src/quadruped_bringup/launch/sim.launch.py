import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 런칭 인자 선언
    use_mcu = LaunchConfiguration('use_mcu')
    mcu_port = LaunchConfiguration('mcu_port')

    declare_use_mcu_arg = DeclareLaunchArgument(
        'use_mcu', default_value='false',
        description='Enable MCU hardware bridge'
    )
    declare_mcu_port_arg = DeclareLaunchArgument(
        'mcu_port', default_value='/dev/ttyUSB0',
        description='Serial port for MCU'
    )

    pkg_description = get_package_share_directory('quadruped_description')
    pkg_bringup = get_package_share_directory('quadruped_bringup')
    pkg_control = get_package_share_directory('quadruped_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_description, 'urdf', 'quadruped.urdf.xacro')

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': os.path.join(pkg_bringup, 'world', 'empty.world'), 'verbose': 'true'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # hip_z=BODY_H/2=0.04m, body_height=0.17m → 발 z=-0.13m → 토구 z≈-0.15m → spawn 0.20m
        arguments=['-topic', 'robot_description', '-entity', 'quadruped', '-z', '0.20'],
        output='screen'
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    load_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller']
    )

    # 실제 로봇 치수와 동일: L1=0.03m, L2=0.115m, L3=0.135m (max_reach=0.25m)
    # V6 BezierGait 파라미터 (hardware.launch.py 와 동일)
    gait_node = Node(
        package='quadruped_gait',
        executable='gait_node',
        name='gait_node',
        parameters=[{
            'use_sim_time': True,
            'L1': 0.030,
            'L2': 0.115,
            'L3': 0.135,
            'body_height': 0.17,
            'step_height': 0.03,    # ClearanceHeight
            'max_stride':  0.025,   # StepLength 상한 (보폭 작게 → 차분한 속도)
            'period':      1.0,     # 전체 cycle (Tswing = 0.5s)
            'height_min':  0.07,    # 앉기 자세 가능 높이 (다리 많이 굽힘)
            'height_max':  0.21,
            'gait_type':   'trot',  # BezierGait 는 trot 만 지원
            'cmd_vel_hold_time': 30.0,
        }]
    )

    # 실제 MCU 연결용 브릿지 노드 (조건부 실행)
    mcu_bridge_node = Node(
        package='quadruped_control',
        executable='mcu_bridge.py',
        name='mcu_bridge',
        parameters=[{'port': mcu_port, 'baudrate': 115200}],
        condition=IfCondition(use_mcu)
    )

    cleanup = ExecuteProcess(cmd=['pkill', '-9', 'gzserver'], output='screen')

    return LaunchDescription([
        declare_use_mcu_arg,
        declare_mcu_port_arg,
        cleanup,
        TimerAction(period=2.0, actions=[robot_state_publisher, gazebo]),
        TimerAction(period=10.0, actions=[spawn_entity]),
        TimerAction(period=18.0, actions=[load_jsb]),
        TimerAction(period=22.0, actions=[load_jtc]),
        TimerAction(period=25.0, actions=[gait_node]),
        # 하드웨어 브릿지는 제어기가 활성화된 후 실행
        TimerAction(period=27.0, actions=[mcu_bridge_node])
    ])
