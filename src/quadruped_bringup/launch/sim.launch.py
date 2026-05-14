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
        # body_height=0.26 → 발이 베이스 링크 기준 -0.16m → 지면 접지 여유 포함 0.30m
        arguments=['-topic', 'robot_description', '-entity', 'quadruped', '-z', '0.30'],
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

    # URDF 치수: shoulder_offset=0.08m, thigh=0.2m, calf=0.2m (max_reach=0.4m)
    gait_node = Node(
        package='quadruped_gait',
        executable='gait_node',
        name='gait_node',
        parameters=[{
            'use_sim_time': True,
            'L1': 0.08,
            'L2': 0.20,
            'L3': 0.20,
            'body_height': 0.26,
            'step_height': 0.06,
            'max_stride':  0.05,
            'period':      1.2,
            'height_min':  0.18,
            'height_max':  0.30,
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
