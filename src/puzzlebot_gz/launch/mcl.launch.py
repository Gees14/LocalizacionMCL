"""
MCL simulation launch — maze world (Gazebo Fortress / ignition-gazebo 6).
Mirrors sim.launch.py exactly; only the world file, bridge topic for
joint_state, and the MCL node are different.

Usage:
  ros2 launch puzzlebot_gz mcl.launch.py
  ros2 launch puzzlebot_gz mcl.launch.py rviz:=false

Teleop (separate terminal):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/model/puzzlebot/cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             TimerAction, IncludeLaunchDescription,
                             SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_pkg     = get_package_share_directory('puzzlebot_gz')
    ctrl_pkg   = get_package_share_directory('puzzlebot_control')
    desc_pkg   = get_package_share_directory('puzzlebot_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file  = os.path.join(gz_pkg, 'urdf', 'puzzlebot_gz.urdf')
    sdf_file   = os.path.join(gz_pkg, 'urdf', 'puzzlebot_gz.sdf')
    world_file = os.path.join(gz_pkg, 'worlds', 'maze.sdf')
    rviz_file  = os.path.join(gz_pkg, 'rviz', 'mcl_rviz.rviz')
    map_file   = os.path.join(ctrl_pkg, 'puzzlebot_control', 'maze_map.png')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # IGN_GAZEBO_RESOURCE_PATH: parent of puzzlebot_description share dir
    desc_share_parent = os.path.dirname(desc_pkg)
    existing_res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_resource_path = desc_share_parent + (':' + existing_res if existing_res else '')

    # ── Launch arguments ──────────────────────────────────────────────────
    arg_rviz = DeclareLaunchArgument('rviz', default_value='true')
    rviz_en  = LaunchConfiguration('rviz')

    # Step 0 — set resource path before any process starts (MUST be first)
    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=ign_resource_path,
    )

    # ── 1. Gazebo Fortress ────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'gz_version': '6',
        }.items(),
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # ── 3. ros_gz_bridge (Fortress: ignition.msgs types, world = "maze") ─
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/model/puzzlebot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/puzzlebot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/world/maze/model/puzzlebot/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        parameters=[{
            'qos_overrides./model/puzzlebot.subscriber.reliability': 'reliable',
        }],
        output='screen',
    )

    # Relay joint_state to /joint_states so robot_state_publisher gets wheel TF
    joint_relay = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_relay',
        arguments=[
            '/world/maze/model/puzzlebot/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/world/maze/model/puzzlebot/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    # ── 4. Spawn robot (5 s delay, Fortress CLI: ign service) ────────────
    spawn = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ign', 'service',
                    '-s', '/world/maze/create',
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

    # Static TF: Fortress scoped lidar name → URDF lidar_link (zero offset)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_fix',
        arguments=['0', '0', '0', '0', '0', '0',
                   'lidar_link', 'puzzlebot/base_footprint/lidar'],
        output='screen',
    )

    # ── 5. dead_reckoning — remapped to maze world joint_state topic ──────
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
            ('/joint_states', '/world/maze/model/puzzlebot/joint_state'),
        ],
    )

    # ── 6. MCL node ───────────────────────────────────────────────────────
    mcl = Node(
        package='puzzlebot_control',
        executable='mcl',
        name='mcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_path':       map_file,
            'map_resolution': 0.05,
            'map_origin_x':  -5.54,
            'map_origin_y':  -8.10,
            'num_particles':  500,
            'top_k':          150,
            'noise_xy':       0.05,
            'noise_theta':    0.05,
            'score_rays':     36,
        }],
    )

    # ── 7. RViz — delayed 10 s so /clock is stable before it starts ──────
    # Starting RViz before the Gazebo clock settles causes a storm of
    # "jump back in time" warnings that reset TF and break all displays.
    rviz = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
        condition=IfCondition(rviz_en),
    )

    return LaunchDescription([
        set_resource_path,   # MUST be first
        arg_rviz,
        gz_sim,
        rsp,
        bridge,
        joint_relay,
        lidar_tf,
        spawn,
        dead_reckoning,
        mcl,
        rviz,
    ])
