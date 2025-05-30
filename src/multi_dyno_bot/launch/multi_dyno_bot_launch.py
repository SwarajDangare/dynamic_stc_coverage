import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    ws        = os.getenv('AMENT_PREFIX_PATH').split(':')[0]
    pkg_share = os.path.join(ws, 'share', 'multi_dyno_bot')
    world_file = os.path.join(pkg_share, 'worlds', 'square_platform.world')

    # 1) Load your custom world in Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
            f'world:={world_file}', 'verbose:=true'
        ],
        output='screen'
    )

    # 2) Spawn TurtleBot3 Burger (with LiDAR) at (0,0)
    spawn_tb3 = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_burger',
                '-file', os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                    'share', 'turtlebot3_gazebo', 'models',
                    'turtlebot3_burger', 'model.sdf'
                ),
                '-x', '0', '-y', '0', '-z', '0.01'
            ],
            output='screen'
        )]
    )

    # 3) Start the blue‐box mover
    dynobs = TimerAction(
        period=6.0,
        actions=[Node(
            package='multi_dyno_bot', executable='random_mover',
            name='random_mover', output='screen'
        )]
    )

    # 4) Start the STC coverage controller
    stc = TimerAction(
        period=7.0,
        actions=[Node(
            package='multi_dyno_bot', executable='stc_controller',
            name='stc_controller', output='screen'
        )]
    )

    visualizer = Node(
        package='multi_dyno_bot', executable='coverage_visualizer',
        name='coverage_visualizer', output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_tb3,
        dynobs,
        stc,
        visualizer,
    ])
