# Puzzlebot ROS 2 + Gazebo Workspace

This workspace contains a Puzzlebot simulation and localization stack built on:

- ROS 2 Humble
- Gazebo Fortress (`ign gazebo`, `ignition-gazebo 6`)
- `ros_gz_bridge` for ROS 2 <-> Gazebo transport bridging

The repository is split into three packages:

- `src/puzzlebot_description`: robot meshes, URDF, and base RViz config
- `src/puzzlebot_gz`: Gazebo world files, simulation launch files, SDF model, and MCL RViz config
- `src/puzzlebot_control`: dead reckoning and Monte Carlo localization nodes

## Important note

This workspace is configured for the Fortress-era bridge described in
[INTEGRATION.md](/home/jesus/Documents/pzGz_ws/INTEGRATION.md). Do not mix the
Fortress bridge with Harmonic `gz sim` binaries or `gz.msgs.*` topic types.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/pzGz_ws
colcon build
source install/setup.bash
```

## Run the flat simulation

```bash
ros2 launch puzzlebot_gz sim.launch.py
```

Teleop from a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/pzGz_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/model/puzzlebot/cmd_vel
```

## Run the maze + MCL demo

```bash
ros2 launch puzzlebot_gz mcl.launch.py
```

This launch file starts:

- the maze world
- the Fortress bridge
- `robot_state_publisher`
- the dead-reckoning odometry node
- the MCL node
- RViz with the localization layout

## Reading the MCL RViz view

- `Map (MCL)`: occupancy map used by the filter
- `LaserScan (raw)`: current LiDAR hits
- `MCL Particles`: particle cloud
- `MCL Best Pose`: mean pose of the strongest particles
- `Odometry (dead reckoning)`: raw odom path before MCL correction

For localization checks, use `map` as the RViz fixed frame. In `map` frame, the
scan should stay aligned with walls and obstacles while driving. In `odom`
frame, the map may appear to rotate or shift as MCL corrects odometry drift.

## Current MCL behavior

The MCL node currently:

- samples particles uniformly in free space
- moves particles with dead-reckoning odometry
- scores particles by comparing measured ranges against ray-marched expected
  ranges from the occupancy map
- publishes a `map -> odom` correction transform

Relevant files:

- [src/puzzlebot_control/puzzlebot_control/mcl.py](/home/jesus/Documents/pzGz_ws/src/puzzlebot_control/puzzlebot_control/mcl.py)
- [src/puzzlebot_control/puzzlebot_control/dead_reckoning.py](/home/jesus/Documents/pzGz_ws/src/puzzlebot_control/puzzlebot_control/dead_reckoning.py)
- [src/puzzlebot_gz/launch/mcl.launch.py](/home/jesus/Documents/pzGz_ws/src/puzzlebot_gz/launch/mcl.launch.py)
- [src/puzzlebot_gz/rviz/mcl_rviz.rviz](/home/jesus/Documents/pzGz_ws/src/puzzlebot_gz/rviz/mcl_rviz.rviz)

## More detail

For the full integration rationale, troubleshooting notes, bridge details, and
Gazebo/ROS 2 version constraints, read
[INTEGRATION.md](/home/jesus/Documents/pzGz_ws/INTEGRATION.md).
