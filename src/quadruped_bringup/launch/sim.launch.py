import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('quadruped_description')
    pkg_bringup = get_package_share_directory('quadruped_bringup')
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
        arguments=['-topic', 'robot_description', '-entity', 'quadruped', '-z', '0.25'],
        output='screen'
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    gait_node = Node(
        package='quadruped_gait',
        executable='gait_node',
        name='gait_node',
        parameters=[{'use_sim_time': True}]
    )

    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--ros-args', '-p', 'use_sim_time:=True']

    cleanup = ExecuteProcess(cmd=['pkill', '-9', 'gzserver'], output='screen')

    return LaunchDescription([
        cleanup,
        # 충분한 간격을 두어 Humble 가제보 통신 안정성 확보
        TimerAction(period=5.0, actions=[robot_state_publisher, gazebo]),
        TimerAction(period=15.0, actions=[spawn_entity]),
        TimerAction(period=25.0, actions=[load_jsb]),
        TimerAction(period=30.0, actions=[load_jtc]),
        TimerAction(period=35.0, actions=[gait_node])
    ])
