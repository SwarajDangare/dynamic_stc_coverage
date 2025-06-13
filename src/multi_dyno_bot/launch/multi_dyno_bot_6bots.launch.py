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

    # --- Optional argument in case you want to gate-drive nodes under each namespace ---
    enable_drive = LaunchConfiguration('enable_drive', default='true')
    ld.add_action(DeclareLaunchArgument(
        'enable_drive',
        default_value='true',
        description='Enable robot drive / STC controller nodes'
    ))

    # --- Launch Gazebo server & client using your custom world file ---
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(
        get_package_share_directory('multi_dyno_bot'),
        'worlds', 'square_platform.world'
    )
    # gzserver (loads world, plugins, etc.)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    ))
    # gzclient (the GUI)
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

    # --- Define the six robot namespaces & EXACT spawn (x,y) so that each lies at center of its 20×30 block ---
    #    See partitioning in the explanation above.
    robot_positions = [
        ('tb1', -14.75, -14.75),   # Block 2 (i=40..59, j=30..59)
        ('tb2', -14.75, -4.75),   # Block 1 (i=20..39, j=30..59)
        ('tb3', -14.75,  5.25),   # Block 0 (i= 0..19, j=30..59)
        ('tb4',   0.25, -14.75),   # Block 5 (i=40..59, j= 0..29)
        ('tb5',   0.25, -4.75),   # Block 4 (i=20..39, j= 0..29)
        ('tb6',   0.25, 5.25),   # Block 3 (i= 0..19, j= 0..29)
    ]

    # --- For each robot, chain spawn + STCController under its namespace ---
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

        # 3) STC controller under the same namespace, with that robot’s 20×30 bounds
        #    (sub_imin…sub_jmax correspond to table above)
        if name == 'tb1':
            sub_args = {'sub_imin': 0, 'sub_imax': 29, 'sub_jmin': 0, 'sub_jmax': 19}
        elif name == 'tb2':
            sub_args = {'sub_imin': 0, 'sub_imax': 29, 'sub_jmin': 20, 'sub_jmax': 39}
        elif name == 'tb3':
            sub_args = {'sub_imin': 0, 'sub_imax': 29, 'sub_jmin': 40, 'sub_jmax': 59}
        elif name == 'tb4':
            sub_args = {'sub_imin': 30, 'sub_imax': 59, 'sub_jmin': 0,  'sub_jmax': 19}
        elif name == 'tb5':
            sub_args = {'sub_imin': 30, 'sub_imax': 59, 'sub_jmin': 20,  'sub_jmax': 39}
        elif name == 'tb6':
            sub_args = {'sub_imin': 30, 'sub_imax': 59, 'sub_jmin': 40,  'sub_jmax': 59}

        stc = Node(
            package='multi_dyno_bot',
            namespace=ns,
            executable='multi_stc_controller',
            name='multi_stc_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'grid_size':   60},
                {'cell_size':   0.5},
                {'robot_list':  ['tb1','tb2','tb3','tb4','tb5','tb6']},
                sub_args,
            ]
        )

        # Chain them so each namespace’s STCController only starts after spawn finishes
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
        # Start that robot’s STCController as soon as its spawn is done
        ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[stc]
            )
        ))

        last_spawn = spawn

    # --- Finally, start random_mover (to move the 10 blue boxes) and the global coverage_visualizer ---
    mover = Node(
        package='multi_dyno_bot',
        executable='random_mover',
        name='random_mover',
        output='screen'
    )
    viz = Node(
        package='multi_dyno_bot',
        executable='coverage_visualizer',
        name='coverage_visualizer',
        output='screen'
    )

    # Only launch these once ALL six robots have been spawned
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_spawn,
            on_exit=[mover, viz]
        )
    ))

    return ld
