from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('robot_bringup'), 'urdf', 'robot_complete.urdf.xacro']
            )
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get config file
    config_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'diff_drive_controller.yaml']
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_file],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Diff Drive Controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay diff_drive_controller after joint_state_broadcaster
    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'config', 'robot.rviz']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Teleop Twist Keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -T "Robot Teleop Keyboard" -geometry 100x30 -e',
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    # Debug processes - rostopic echo en ventanas separadas
    joint_states_echo = ExecuteProcess(
        cmd=['xterm', 
             '-T', 'Joint States Monitor', 
             '-geometry', '100x30+50+50',
             '-fa', 'Monospace', '-fs', '12',
             '-e', 'ros2', 'topic', 'echo', '/joint_states'],
        output='screen'
    )

    cmd_vel_echo = ExecuteProcess(
        cmd=['xterm',
             '-T', 'Velocity Commands Monitor',
             '-geometry', '100x30+150+150',
             '-fa', 'Monospace', '-fs', '12',
             '-e', 'ros2', 'topic', 'echo', '/diff_drive_controller/cmd_vel_unstamped'],
        output='screen'
    )

    tf_echo = ExecuteProcess(
        cmd=['xterm',
             '-T', 'Transform Monitor',
             '-geometry', '100x30+250+250',
             '-fa', 'Monospace', '-fs', '12',
             '-e', 'ros2', 'topic', 'echo', '/tf'],
        output='screen'
    )

    nodes_and_processes = [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,
        rviz_node,
        teleop_node,
        joint_states_echo,
        cmd_vel_echo,
        tf_echo
    ]

    return LaunchDescription(nodes_and_processes) 