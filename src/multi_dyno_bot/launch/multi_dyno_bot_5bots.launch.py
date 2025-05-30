#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()

    # --- optional arg if you later want to gate-drive nodes under each namespace ---
    enable_drive = LaunchConfiguration('enable_drive', default='true')
    ld.add_action(DeclareLaunchArgument(
        'enable_drive', default_value='true',
        description='Enable robot drive / STC controller nodes'
    ))

    # --- Gazebo server & client on your custom world ---
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(
        get_package_share_directory('multi_dyno_bot'),
        'worlds', 'square_platform.world'
    )
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ),
    ))

    # --- Paths to TurtleBot3 URDF & SDF model ---
    tb3_desc_pkg = get_package_share_directory('turtlebot3_description')
    urdf_file = os.path.join(tb3_desc_pkg, 'urdf', 'turtlebot3_burger.urdf')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    model_sdf = os.path.join(
        tb3_gazebo_pkg, 'models', 'turtlebot3_burger', 'model.sdf'
    )

    # --- Define your five robot namespaces & start poses ---
    robot_positions = [
        ('tb1', -4.5, -4.5),
        ('tb2',  4.5, -4.5),
        ('tb3', -4.5,  4.5),
        ('tb4',  4.5,  4.5),
        ('tb5',  0.0,  0.0),
    ]

    last_spawn = None
    for name, x, y in robot_positions:
        ns = name  # namespace & entity name

        # 1) robot_state_publisher under /<ns>
        rsp = Node(
            package='robot_state_publisher',
            namespace=ns,
            executable='robot_state_publisher',
            name='rsp',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[urdf_file]
        )

        # 2) spawn_entity.py under /<ns>
        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            namespace=ns,
            name=f'spawn_{name}',
            output='screen',
            arguments=[
                '-file',           model_sdf,
                '-entity',         name,
                '-robot_namespace', f'/{ns}',
                '-x',              str(x),
                '-y',              str(y),
                '-z',              '0.01',
            ],
        )

        # Chain so each robot is spawned only after the previous one finishes
        if last_spawn is None:
            ld.add_action(rsp)
            ld.add_action(spawn)
        else:
            ld.add_action(RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_spawn,
                    on_exit=[rsp, spawn]
                )
            ))
        last_spawn = spawn

    # --- After all robots are spawned, start your mover, STC, and visualizer ---
    mover = Node(
        package='multi_dyno_bot',
        executable='random_mover',
        name='random_mover',
        output='screen'
    )
    stc   = Node(
        package='multi_dyno_bot',
        executable='stc_controller',
        name='stc_controller',
        output='screen'
    )
    viz   = Node(
        package='multi_dyno_bot',
        executable='coverage_visualizer',
        name='coverage_visualizer',
        output='screen'
    )

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_spawn,
            on_exit=[mover, stc, viz]
        )
    ))

    return ld
