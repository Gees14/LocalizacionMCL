"""
Puzzlebot — Gazebo Fortress (ignition-gazebo 6) simulation launch.
Uses ros-humble-ros-gz (Fortress bridge) which is the official Humble pairing.

Usage:
  ros2 launch puzzlebot_gz sim.launch.py
  ros2 launch puzzlebot_gz sim.launch.py rviz:=false

Teleop (separate terminal):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/model/puzzlebot/cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, TimerAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_pkg     = get_package_share_directory('puzzlebot_gz')
    desc_pkg   = get_package_share_directory('puzzlebot_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file  = os.path.join(gz_pkg, 'urdf', 'puzzlebot_gz.urdf')
    sdf_file   = os.path.join(gz_pkg, 'urdf', 'puzzlebot_gz.sdf')
    world_file = os.path.join(gz_pkg, 'worlds', 'flat_plane.sdf')
    rviz_file  = os.path.join(desc_pkg, 'rviz', 'puzzlebot_rviz.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # IGN_GAZEBO_RESOURCE_PATH: parent of puzzlebot_description share dir
    # so Gazebo resolves model://puzzlebot_description/meshes/...
    desc_share_parent = os.path.dirname(desc_pkg)
    existing_res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_resource_path = desc_share_parent + (':' + existing_res if existing_res else '')

    # ── Launch arguments ──────────────────────────────────────────────
    arg_rviz = DeclareLaunchArgument('rviz', default_value='true')
    rviz_en  = LaunchConfiguration('rviz')

    # Set IGN_GAZEBO_RESOURCE_PATH in the process environment BEFORE Gazebo
    # launches so the server and GUI both inherit it and can resolve
    # model://puzzlebot_description/meshes/... URIs from the SDF.
    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=ign_resource_path,
    )

    # ── 1. Gazebo Fortress via gz_sim.launch.py ───────────────────────
    # ign_args triggers the Fortress code path (ruby ign gazebo)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'gz_version': '6',
        }.items(),
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # ── 3. ros_gz_bridge (Fortress: ignition.msgs types) ─────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/model/puzzlebot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/puzzlebot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/world/flat_plane/model/puzzlebot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        parameters=[{
            'qos_overrides./model/puzzlebot.subscriber.reliability': 'reliable',
        }],
        output='screen',
    )

    # ── 4. Spawn robot (5 s delay) ────────────────────────────────────
    spawn = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ign', 'service',
                    '-s', '/world/flat_plane/create',
                    '--reqtype', 'ignition.msgs.EntityFactory',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '5000',
                    '--req',
                    f'sdf_filename: "{sdf_file}", name: "puzzlebot", '
                    f'pose: {{position: {{z: 0.05}}}}',
                ],
                additional_env={'IGN_GAZEBO_RESOURCE_PATH': ign_resource_path},
                output='screen',
            )
        ],
    )

    # ── 5. dead_reckoning ─────────────────────────────────────────────
    dead_reckoning = Node(
        package='puzzlebot_control',
        executable='dead_reckoning',
        name='dead_reckoning',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wheel_radius': 0.05,
            'wheel_separation': 0.19,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
        remappings=[
            ('/joint_states', '/world/flat_plane/model/puzzlebot/joint_state'),
        ],
    )

    # ── 6. RViz2 ──────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz_en),
        output='screen',
    )

    return LaunchDescription([
        set_resource_path,
        arg_rviz,
        gz_sim,
        rsp,
        bridge,
        spawn,
        dead_reckoning,
        rviz,
    ])
