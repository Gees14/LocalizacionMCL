# Puzzlebot — Monte Carlo Localization

ROS 2 Humble workspace that implements **Monte Carlo Localization (MCL)** for the
Puzzlebot differential-drive robot, simulated in Gazebo Fortress.

**Author:** Jorge Reyes — `rpz.dar14@gmail.com`

---

## Stack

| Layer | Version |
|-------|---------|
| ROS 2 | Humble |
| Gazebo | Fortress (ignition-gazebo 6) |
| Bridge | ros-humble-ros-gz (ignition.msgs) |

---

## Package layout

```
src/
├── puzzlebot_description/   # URDF, meshes, RViz configs
├── puzzlebot_gz/            # SDF worlds, launch files
└── puzzlebot_control/       # Algorithm nodes
    └── puzzlebot_control/
        ├── map_loader.py        — PNG occupancy map + ray casting
        ├── particle_filter.py   — MCL particle filter (pure Python, no ROS)
        ├── localization_node.py — ROS 2 node: /odom + /scan → /localization/*
        ├── wheel_odometry.py    — Differential-drive odometry node
        └── maze_builder.py      — Generates maze_map.png from geometry
```

---

## Build

```bash
cd ~/pzGz_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Run

### Flat-plane simulation (odometry only)

```bash
ros2 launch puzzlebot_gz sim.launch.py
```

### MCL in the maze world

```bash
ros2 launch puzzlebot_gz mcl.launch.py
```

### Teleop (separate terminal)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/model/puzzlebot/cmd_vel
```

---

## Topics published by the localization node

| Topic | Type | Description |
|-------|------|-------------|
| `/localization/particles` | `geometry_msgs/PoseArray` | Full particle cloud |
| `/localization/pose` | `geometry_msgs/PoseStamped` | Weighted mean estimate |
| `/localization/map` | `nav_msgs/OccupancyGrid` | Static occupancy map |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |

TF tree: `map → odom → base_footprint`

---

## RViz displays

Add these displays in RViz after launching `mcl.launch.py`:

| Display | Topic / Frame |
|---------|---------------|
| PoseArray | `/localization/particles` |
| PoseStamped | `/localization/pose` |
| Map | `/localization/map` |
| Odometry | `/odom` |
| LaserScan | `/scan` |
| TF | — |

---

## Regenerate the map

If you change the maze geometry in `maze_builder.py`, regenerate the PNG:

```bash
ros2 run puzzlebot_control maze_builder
```

---

## Algorithm overview

1. **Seed** — N particles placed uniformly in free space
2. **Predict** — each particle moved by odometry delta + Gaussian noise (motion model)
3. **Update** — each particle scored via Gaussian beam likelihood against laser scan (sensor model)
4. **Resample** — top-K survivors cloned back to N with small perturbation
5. **Estimate** — weighted circular mean of all particles

See `src/puzzlebot_control/puzzlebot_control/particle_filter.py` for the implementation.
