# Puzzlebot — ROS 2 + Gazebo Integration Guide

Complete reference for the `pzGz_ws` workspace: how every piece fits together,
why specific decisions were made, what went wrong during development, and how to
extend the setup for future work.

---

## Table of Contents

1. [Stack Overview](#1-stack-overview)
2. [Critical Lesson: The Mixed-Gazebo Trap](#2-critical-lesson-the-mixed-gazebo-trap)
3. [Workspace Layout](#3-workspace-layout)
4. [Key Concepts: Two Middleware Layers](#4-key-concepts-two-middleware-layers)
5. [Topic Bridge — The Core of the Integration](#5-topic-bridge--the-core-of-the-integration)
6. [SDF Model — Gazebo Plugin Configuration](#6-sdf-model--gazebo-plugin-configuration)
7. [World File](#7-world-file)
8. [Launch File Walkthrough](#8-launch-file-walkthrough)
9. [Dead Reckoning Node](#9-dead-reckoning-node)
10. [Building the Workspace](#10-building-the-workspace)
11. [Running the Simulation](#11-running-the-simulation)
12. [Teleop](#12-teleop)
13. [Verifying Everything Works](#13-verifying-everything-works)
14. [Sim → Real Robot Transition](#14-sim--real-robot-transition)
15. [Common Errors and Fixes](#15-common-errors-and-fixes)
16. [Adding New Bridged Topics](#16-adding-new-bridged-topics)

---

## 1. Stack Overview

| Layer | Software | Version |
|---|---|---|
| OS | Ubuntu 22.04 | — |
| Middleware | ROS 2 | Humble |
| Simulator | Gazebo | **Fortress (ignition-gazebo 6)** |
| Bridge | ros_gz_bridge | Humble (`ros-humble-ros-gz`) |

**Why Fortress and not Harmonic?**

This is the most important design decision in the workspace. See
[Section 2](#2-critical-lesson-the-mixed-gazebo-trap) for the full story.
The short answer: `ros-humble-ros-gz` from `packages.ros.org` installs the
**Fortress bridge**. Attempting to use Gazebo Harmonic (`gz-sim 8`) alongside
it causes a silent ABI mismatch that breaks the DiffDrive plugin subscriber —
the robot spawns but never responds to any velocity commands.

---

## 2. Critical Lesson: The Mixed-Gazebo Trap

> **Read this before touching any Gazebo version or package.**

### What happened

The workspace was initially developed targeting Gazebo Harmonic (`gz-sim 8`,
the newest LTS). The SDF files used Harmonic plugin names (`gz-sim-diff-drive-system`,
`gz::sim::systems::DiffDrive`) and the bridge used Harmonic message types
(`gz.msgs.Twist`). The launch file started `gz sim`.

Everything appeared to work — Gazebo opened, the world loaded, the robot
spawned — but the robot would never move regardless of what was published to
`/model/puzzlebot/cmd_vel`.

Diagnostics revealed the root cause:

```bash
ign topic -i -t /model/puzzlebot/cmd_vel
# Publishers:   tcp://172.17.0.1:46545, ignition.msgs.Twist   ← GUI process
# Subscribers:  tcp://172.17.0.1:34137, gz.msgs.Twist          ← bridge
# DiffDrive subscriber: MISSING
```

The DiffDrive plugin subscriber was not registering at all. The bridge was
subscribing on the ROS 2 side and connecting to gz transport, but its message
type (`gz.msgs.Twist`) did not match what the Fortress-era DiffDrive plugin
expected (`ignition.msgs.Twist`). The plugin's `Configure()` was failing
silently, so no subscriber was ever created.

### Why it happened

When you run `sudo apt install ros-humble-ros-gz` on Ubuntu 22.04, you get the
**Fortress-era bridge** from `packages.ros.org`. This is the **official**
ROS 2 Humble pairing.

Gazebo Harmonic (`gz-harmonic`, from `packages.osrfoundation.org`) was also
installed. Both packages coexist on disk, but they use incompatible transport
ABIs:

| Package | Transport namespace | Binary |
|---|---|---|
| `ros-humble-ros-gz` (Fortress bridge) | `ignition.msgs` | `ign gazebo` |
| `gz-harmonic` | `gz.msgs` | `gz sim` |

The launch file was starting the **Harmonic** binary (`gz sim`) but the bridge
was linked against the **Fortress** libraries (`ignition.msgs`). Fortress
DiffDrive plugin exists at
`/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-diff-drive-system.so`
and expects `ignition.msgs.Twist`. The Harmonic-style plugin name
`gz-sim-diff-drive-system` with `gz::sim::` namespace was being loaded from the
Harmonic installation which uses `gz.msgs.Twist`. The bridge and the plugin
were not in the same transport universe.

### The fix and the rule

**Rule: match the bridge version to the Gazebo binary being launched.**

On ROS 2 Humble with the standard apt packages:
- Bridge installed: Fortress (`ros-humble-ros-gz`)
- Correct Gazebo binary: `ign gazebo` (Fortress, `ignition-gazebo 6`)
- Plugin filenames: `libignition-gazebo-*.so`
- Plugin class names: `ignition::gazebo::systems::*`
- Message types: `ignition.msgs.*`
- Resource path env var: `IGN_GAZEBO_RESOURCE_PATH`
- Spawn service CLI: `ign service`

If you ever upgrade to ROS 2 Jazzy, the correct pairing flips to Harmonic:
- Bridge: `ros-jazzy-ros-gz` (Harmonic bridge)
- Correct Gazebo binary: `gz sim` (Gazebo Harmonic 8)
- Plugin filenames: `gz-sim-diff-drive-system` (no `.so`, no `lib` prefix)
- Plugin class names: `gz::sim::systems::*`
- Message types: `gz.msgs.*`
- Resource path env var: `GZ_SIM_RESOURCE_PATH`
- Spawn service CLI: `gz service`

Never mix binaries and bridges from different Gazebo generations.

### How to detect a mismatch

Run the simulation, spawn the robot, then:

```bash
ign topic -i -t /model/puzzlebot/cmd_vel
```

A healthy output shows **two entries under Subscribers** — one from the bridge,
one from the DiffDrive plugin. If only one subscriber appears (or none), the
plugin failed to initialise, almost certainly due to a version mismatch.

---

## 3. Workspace Layout

```
pzGz_ws/
├── src/
│   ├── puzzlebot_description/   # URDF + meshes (read-only, not modified)
│   │   ├── meshes/              # .stl files referenced by SDF
│   │   └── urdf/puzzlebot.urdf
│   │
│   ├── puzzlebot_gz/            # Simulation-specific: world, SDF, launch
│   │   ├── launch/sim.launch.py
│   │   ├── urdf/
│   │   │   ├── puzzlebot_gz.urdf   # URDF (used by robot_state_publisher)
│   │   │   └── puzzlebot_gz.sdf    # Hand-authored SDF (used by Gazebo)
│   │   ├── worlds/flat_plane.sdf
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── puzzlebot_control/       # Algorithm nodes (sim + real)
│       ├── puzzlebot_control/
│       │   └── dead_reckoning.py
│       ├── setup.py
│       ├── setup.cfg
│       ├── pyproject.toml       # Required for setuptools ≥ 64
│       └── package.xml
│
├── install/                     # colcon output (do not edit)
└── build/                       # colcon intermediates (do not edit)
```

**Package roles:**

- `puzzlebot_description` — robot geometry and meshes only. No nodes, no launch.
- `puzzlebot_gz` — everything Gazebo-specific: the world, the SDF model, the
  bridge, and the simulation launch file.
- `puzzlebot_control` — algorithm nodes that run identically in sim and on the
  real robot. No Gazebo dependency.

---

## 4. Key Concepts: Two Middleware Layers

```
┌─────────────────────────────┐     ┌──────────────────────────────┐
│        ROS 2 (DDS)          │     │  Gazebo Fortress (ign transport)│
│                             │     │                               │
│  teleop → /model/puzzlebot/ │     │  DiffDrive plugin listens on  │
│           cmd_vel           │     │  /model/puzzlebot/cmd_vel     │
│                             │     │                               │
│  dead_reckoning subscribes  │     │  JointStatePublisher pub on   │
│  /world/.../joint_state     │     │  /world/.../joint_state       │
│                             │     │                               │
└──────────┬──────────────────┘     └──────────────────┬───────────┘
           │                                           │
           └──────────── ros_gz_bridge ────────────────┘
                    (parameter_bridge, ignition.msgs types)
```

**ROS 2 uses DDS** — topics discovered via UDP multicast.

**Gazebo Fortress uses ign transport** — a completely separate pub/sub system
built on ZeroMQ. Not DDS. Topics on the ign side are invisible to ROS 2 nodes
and vice versa.

**`ros_gz_bridge` (`parameter_bridge`)** is the only component that speaks both
languages. Every topic that must cross between ROS 2 and Gazebo must be
explicitly declared in the bridge with the correct message type namespace
(`ignition.msgs.*` for Fortress).

---

## 5. Topic Bridge — The Core of the Integration

### Bridge argument syntax

```
/topic_name@ros_type@gz_type
```

The `@` delimiters encode direction:

| Format | Direction |
|---|---|
| `/topic@ros_type@gz_type` | Bidirectional |
| `/topic@ros_type[gz_type` | gz → ROS 2 only |
| `/topic@ros_type]gz_type` | ROS 2 → gz only |

### Bridge topics in this workspace (Fortress / `ignition.msgs`)

```python
arguments=[
    # cmd_vel: ROS 2 (teleop) ↔ Gazebo DiffDrive plugin
    '/model/puzzlebot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',

    # odometry: Gazebo → ROS 2
    '/model/puzzlebot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',

    # clock: Gazebo → ROS 2
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

    # lidar: Gazebo → ROS 2
    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

    # joint states: Gazebo → ROS 2
    '/world/flat_plane/model/puzzlebot/joint_state'
    '@sensor_msgs/msg/JointState[ignition.msgs.Model',
],
```

The message type namespace **must be `ignition.msgs`**, not `gz.msgs`. Using
`gz.msgs` here while running Fortress is what caused the original
DiffDrive-subscriber-missing bug.

### Common message type mappings (Fortress)

| `ignition.msgs` type | ROS 2 type |
|---|---|
| `ignition.msgs.Twist` | `geometry_msgs/msg/Twist` |
| `ignition.msgs.Odometry` | `nav_msgs/msg/Odometry` |
| `ignition.msgs.LaserScan` | `sensor_msgs/msg/LaserScan` |
| `ignition.msgs.Image` | `sensor_msgs/msg/Image` |
| `ignition.msgs.Imu` | `sensor_msgs/msg/Imu` |
| `ignition.msgs.Clock` | `rosgraph_msgs/msg/Clock` |
| `ignition.msgs.Model` | `sensor_msgs/msg/JointState` |

### Verifying bridge topics at runtime

```bash
# List all active ign transport topics
ign topic -l

# Inspect a topic — check publishers AND subscribers
ign topic -i -t /model/puzzlebot/cmd_vel

# Echo a topic directly on ign transport
ign topic -e -t /model/puzzlebot/odometry

# Send a command directly via ign transport (bypasses bridge, tests plugin directly)
ign topic -t /model/puzzlebot/cmd_vel -m ignition.msgs.Twist \
  -p "linear: {x: 0.3}, angular: {z: 0.0}"

# ROS 2 side
ros2 topic list
ros2 topic info /model/puzzlebot/cmd_vel
ros2 topic echo /model/puzzlebot/odometry
```

### QoS override

```python
parameters=[{
    'qos_overrides./model/puzzlebot.subscriber.reliability': 'reliable',
}]
```

Forces the bridge to subscribe as `RELIABLE` rather than `BEST_EFFORT` so
commands are not dropped under load.

---

## 6. SDF Model — Gazebo Plugin Configuration

File: `src/puzzlebot_gz/urdf/puzzlebot_gz.sdf`

> **Do not regenerate this file with `gz sdf -p` or `ign sdf -p`.**
> The converter changes plugin names and loses hand-authored configuration.

### DiffDrive plugin (Fortress naming)

```xml
<plugin filename='libignition-gazebo-diff-drive-system.so'
        name='ignition::gazebo::systems::DiffDrive'>
  <left_joint>wheel_l_joint</left_joint>
  <right_joint>wheel_r_joint</right_joint>
  <wheel_separation>0.19</wheel_separation>
  <wheel_radius>0.05</wheel_radius>
  <max_linear_acceleration>1.0</max_linear_acceleration>
  <max_angular_acceleration>2.0</max_angular_acceleration>
  <max_linear_velocity>0.5</max_linear_velocity>
  <max_angular_velocity>2.0</max_angular_velocity>
  <cmd_vel_timeout>0.5</cmd_vel_timeout>
</plugin>
```

- Filename must be `libignition-gazebo-diff-drive-system.so` (full `.so` name)
- Class must be `ignition::gazebo::systems::DiffDrive`
- `<cmd_vel_timeout>0.5</cmd_vel_timeout>` — robot stops automatically if no
  command arrives within 0.5 s. Without this, closing teleop without pressing
  `k` leaves the robot moving forever.
- No `<topic>` tag → default topic is `/model/puzzlebot/cmd_vel`

### JointStatePublisher plugin (Fortress naming)

```xml
<plugin filename='libignition-gazebo-joint-state-publisher-system.so'
        name='ignition::gazebo::systems::JointStatePublisher'>
  <joint_name>wheel_l_joint</joint_name>
  <joint_name>wheel_r_joint</joint_name>
</plugin>
```

Default topic: `/world/flat_plane/model/puzzlebot/joint_state`

### Wheel joint placement

```xml
<joint name='wheel_l_joint' type='revolute'>
  <pose relative_to='base_footprint'>0.052 0.095 0.05 0 0 0</pose>
```

The z offset (0.05) equals `wheel_radius`. This puts the wheel axle at the
right height so the wheel rests on z=0. If `z < wheel_radius`, the wheel clips
into the ground plane and friction locks the robot permanently.

### Mesh URIs and `IGN_GAZEBO_RESOURCE_PATH`

Meshes use `model://` URIs:

```xml
<uri>model://puzzlebot_description/meshes/Puzzlebot_Wheel.stl</uri>
```

Gazebo resolves `model://puzzlebot_description/` by scanning
`IGN_GAZEBO_RESOURCE_PATH` for a directory named `puzzlebot_description`.
The path must point to the **parent** of the package share directory.

**This variable must be set before Gazebo launches** — it must be an actual
process environment variable, not just passed to the spawn subprocess. The
launch file uses `SetEnvironmentVariable` for this:

```python
desc_share_parent = os.path.dirname(get_package_share_directory('puzzlebot_description'))
set_resource_path = SetEnvironmentVariable(
    name='IGN_GAZEBO_RESOURCE_PATH',
    value=desc_share_parent,
)
```

If you only pass it via `additional_env` to the spawn command, the Gazebo
server and GUI processes never see it, and meshes fail to load (robot is
invisible, shown only as collision boxes or not at all).

---

## 7. World File

File: `src/puzzlebot_gz/worlds/flat_plane.sdf`

World name: `flat_plane` — appears in the joint state topic path
`/world/flat_plane/model/puzzlebot/joint_state`. Renaming the world requires
updating the bridge argument and the dead reckoning remapping.

Required plugins for a functional simulation (Fortress naming):

```xml
<plugin filename="libignition-gazebo-physics-system.so"
        name="ignition::gazebo::systems::Physics"/>
<plugin filename="libignition-gazebo-user-commands-system.so"
        name="ignition::gazebo::systems::UserCommands"/>
<plugin filename="libignition-gazebo-scene-broadcaster-system.so"
        name="ignition::gazebo::systems::SceneBroadcaster"/>
<plugin filename="libignition-gazebo-sensors-system.so"
        name="ignition::gazebo::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="libignition-gazebo-contact-system.so"
        name="ignition::gazebo::systems::Contact"/>
```

Missing `UserCommands` causes the spawn service call to hang indefinitely.
Missing `Sensors` means the lidar produces no data.

---

## 8. Launch File Walkthrough

File: `src/puzzlebot_gz/launch/sim.launch.py`

### Step 0 — Set `IGN_GAZEBO_RESOURCE_PATH` first

```python
set_resource_path = SetEnvironmentVariable(
    name='IGN_GAZEBO_RESOURCE_PATH',
    value=ign_resource_path,   # parent of puzzlebot_description share dir
)
```

This must be the **first action** in the `LaunchDescription` list so the
variable is in the environment before any child process (Gazebo, bridge) starts.

### Step 1 — Launch Gazebo Fortress via the official helper

```python
gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={
        'gz_args': f'-r {world_file}',
        'gz_version': '6',       # 6 = Fortress; triggers ign gazebo code path
    }.items(),
)
```

`gz_version: '6'` causes `gz_sim.launch.py` to use the `ign gazebo` binary
instead of `gz sim`. This sets `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` correctly so
the Fortress physics, sensors, and DiffDrive plugins are found.

`-r` starts the simulation running immediately (not paused).

### Step 2 — robot_state_publisher

```python
rsp = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
)
```

Reads the URDF and publishes `/tf` transforms for fixed joints. Required by
RViz and any node that needs the kinematic tree.

### Step 3 — ros_gz_bridge

```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='gz_bridge',
    arguments=[
        '/model/puzzlebot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/model/puzzlebot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/world/flat_plane/model/puzzlebot/joint_state'
        '@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ],
    parameters=[{
        'qos_overrides./model/puzzlebot.subscriber.reliability': 'reliable',
    }],
)
```

Starts immediately. Waits for gz topics to appear before forwarding.

### Step 4 — Spawn robot (5 s delay)

```python
spawn = TimerAction(
    period=5.0,
    actions=[
        ExecuteProcess(
            cmd=[
                'ign', 'service',           # Fortress CLI: ign, not gz
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
```

Uses `ign service` (Fortress CLI), `ignition.msgs.EntityFactory`, and the
5-second delay to wait for Gazebo to register the create service.

### Step 5 — dead_reckoning

```python
dead_reckoning = Node(
    package='puzzlebot_control',
    executable='dead_reckoning',
    parameters=[{
        'use_sim_time': True,
        'wheel_radius': 0.05,
        'wheel_separation': 0.19,
    }],
    remappings=[
        ('/joint_states', '/world/flat_plane/model/puzzlebot/joint_state'),
    ],
)
```

The remapping redirects the node's internal `/joint_states` subscription to
the actual Gazebo topic, keeping algorithm code free of Gazebo-specific names.

### Full LaunchDescription order

```python
return LaunchDescription([
    set_resource_path,   # MUST be first
    arg_rviz,
    gz_sim,
    rsp,
    bridge,
    spawn,
    dead_reckoning,
    rviz,
])
```

---

## 9. Dead Reckoning Node

File: `src/puzzlebot_control/puzzlebot_control/dead_reckoning.py`

### Kinematics

```
v  = r * (ω_r + ω_l) / 2       [m/s]   linear velocity
ω  = r * (ω_r - ω_l) / L       [rad/s] angular velocity

x   += v * cos(θ) * dt
y   += v * sin(θ) * dt
θ   += ω * dt
```

### Dual input source

| `input_source` param | Topics subscribed | Used in |
|---|---|---|
| `joint_states` (default) | `/joint_states` (→ remapped to Gazebo topic) | Simulation |
| `encoders` | `/velocity_enc_r`, `/velocity_enc_l` (`std_msgs/Float32`) | Real robot |

### Outputs

| Topic | Type |
|---|---|
| `/odom` | `nav_msgs/Odometry` |
| TF `odom → base_footprint` | `tf2` |

---

## 10. Building the Workspace

### First build

```bash
cd ~/Documents/pzGz_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### After any change

```bash
colcon build --packages-select puzzlebot_gz   # or puzzlebot_control
source install/setup.bash
```

> **Always source after building.** The `install/` directory is what the
> runtime reads. Changes are invisible until you source.

### setuptools compatibility (Python packages)

ROS 2 Humble's colcon uses `pip install --editable`, which setuptools ≥ 64
dropped without a `pyproject.toml`. All three files are required:

```toml
# pyproject.toml
[build-system]
requires = ["setuptools", "wheel"]
build-backend = "setuptools.build_meta"
```

---

## 11. Running the Simulation

```bash
cd ~/Documents/pzGz_ws
source install/setup.bash
ros2 launch puzzlebot_gz sim.launch.py
# or without RViz:
ros2 launch puzzlebot_gz sim.launch.py rviz:=false
```

**Launch sequence:**
1. `IGN_GAZEBO_RESOURCE_PATH` is set in the environment
2. Gazebo Fortress starts and loads `flat_plane.sdf` (≈ 3–5 s)
3. `robot_state_publisher` and `ros_gz_bridge` start immediately
4. After 5 s, the robot is spawned via `ign service` with meshes visible
5. `dead_reckoning` begins integrating once joint states arrive

---

## 12. Teleop

In a **separate interactive terminal** (teleop requires a real TTY):

```bash
source ~/Documents/pzGz_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/model/puzzlebot/cmd_vel
```

The `--remap` is mandatory. Without it, teleop publishes to `/cmd_vel` which
nothing subscribes to.

**Key bindings:**

| Key | Action |
|---|---|
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `k` | Stop (send zero) |
| `q` / `z` | Increase / decrease speed |

When you close teleop, the robot stops automatically within 0.5 s due to
`<cmd_vel_timeout>` in the DiffDrive plugin.

---

## 13. Verifying Everything Works

### Topic checklist after robot spawns

```bash
ros2 topic list
```

Expected:
```
/clock
/model/puzzlebot/cmd_vel
/model/puzzlebot/odometry
/odom
/scan
/world/flat_plane/model/puzzlebot/joint_state
/tf
/tf_static
```

### Check clock is flowing

```bash
ros2 topic hz /clock   # should be ~1000 Hz
```

### Send a test command and check odometry moves

```bash
ros2 topic pub --once /model/puzzlebot/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'

ros2 topic echo /model/puzzlebot/odometry --field pose.pose.position
```

The x value should increase by ~0.15 m after one `--once` publish, then stop
within 0.5 s (cmd_vel timeout).

### Check the DiffDrive plugin subscriber is registered

```bash
ign topic -i -t /model/puzzlebot/cmd_vel
```

Must show **two subscribers** — one from the bridge, one from the DiffDrive
plugin. If only one appears, the plugin failed to initialise (version mismatch).

### Send via ign transport directly (bypasses bridge)

```bash
ign topic -t /model/puzzlebot/cmd_vel -m ignition.msgs.Twist \
  -p "linear: {x: 0.3}, angular: {z: 0.0}"
```

If this moves the robot but `ros2 topic pub` does not, the issue is in the
bridge layer (wrong message types or bridge not running).

---

## 14. Sim → Real Robot Transition

Algorithm nodes in `puzzlebot_control` deploy unchanged to the physical robot.
Only the launch configuration differs.

### Real robot topic interface

| Sim topic | Real robot topic | Type |
|---|---|---|
| `/model/puzzlebot/cmd_vel` (via bridge) | `/cmd_vel` | `geometry_msgs/Twist` |
| `/world/.../joint_state` (via bridge) | `/velocity_enc_r`, `/velocity_enc_l` | `std_msgs/Float32` [rad/s] |

### Real robot launch parameters

```python
Node(
    package='puzzlebot_control',
    executable='dead_reckoning',
    parameters=[{
        'input_source': 'encoders',   # switches to Float32 encoder topics
        'use_sim_time': False,
        'wheel_radius': 0.05,
        'wheel_separation': 0.19,
    }],
    # No remappings needed — encoder topics are subscribed directly
)
```

### What stays the same

- `dead_reckoning.py` — identical, no recompile
- `/odom` output — same type and frame IDs
- TF `odom → base_footprint` — identical

---

## 15. Common Errors and Fixes

### Robot spawns but does not move (DiffDrive subscriber missing)

This is the version mismatch bug described in [Section 2](#2-critical-lesson-the-mixed-gazebo-trap).

Verify:
```bash
ign topic -i -t /model/puzzlebot/cmd_vel
```

If only one subscriber is listed, check:
1. SDF plugin filename uses `libignition-gazebo-diff-drive-system.so` (not `gz-sim-diff-drive-system`)
2. Class name is `ignition::gazebo::systems::DiffDrive` (not `gz::sim::`)
3. Bridge argument uses `ignition.msgs.Twist` (not `gz.msgs.Twist`)
4. Launch file uses `gz_version: '6'` (not `'8'`)

---

### Robot keeps moving after teleop is closed

`<cmd_vel_timeout>` was missing from the SDF. Add it inside the DiffDrive plugin:

```xml
<cmd_vel_timeout>0.5</cmd_vel_timeout>
```

Also check for stale background publishers from testing:
```bash
# Kill any leftover ros2 topic pub processes
pkill -f "ros2 topic pub"
```

---

### Robot invisible in Gazebo (meshes not loading)

`IGN_GAZEBO_RESOURCE_PATH` was not set in the Gazebo process environment.

The wrong way (only sets it for the spawn subprocess — Gazebo never sees it):
```python
# BAD: additional_env only affects the spawn ExecuteProcess child
additional_env={'IGN_GAZEBO_RESOURCE_PATH': path}
```

The correct way (sets it for the whole launch, inherited by Gazebo):
```python
# GOOD: SetEnvironmentVariable as the first action
SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=path)
```

The path must be the **parent** of the `puzzlebot_description` share directory:
```python
desc_share_parent = os.path.dirname(
    get_package_share_directory('puzzlebot_description')
)
```

---

### `--editable not recognized` during colcon build

Add `pyproject.toml` to the Python package:
```toml
[build-system]
requires = ["setuptools", "wheel"]
build-backend = "setuptools.build_meta"
```

---

### Spawn hangs at "Requesting list of world names"

`UserCommands` plugin missing from the world file, or using the wrong CLI.
For Fortress use `ign service`, not `gz service`.

World file must contain:
```xml
<plugin filename="libignition-gazebo-user-commands-system.so"
        name="ignition::gazebo::systems::UserCommands"/>
```

---

### Bridge not receiving data from Gazebo (`ros2 topic hz /clock` shows 0)

The bridge is running but not connected. Usually means Gazebo is still starting.
Wait 5–10 s. If it stays at 0, restart the full launch.

---

## 16. Adding New Bridged Topics

### 1. Find the ign transport topic name and type

```bash
ign topic -l
ign topic -i -t /some/topic
```

### 2. Find the ROS 2 type — Fortress mappings

| `ignition.msgs` type | ROS 2 type |
|---|---|
| `ignition.msgs.Twist` | `geometry_msgs/msg/Twist` |
| `ignition.msgs.Odometry` | `nav_msgs/msg/Odometry` |
| `ignition.msgs.LaserScan` | `sensor_msgs/msg/LaserScan` |
| `ignition.msgs.Image` | `sensor_msgs/msg/Image` |
| `ignition.msgs.Imu` | `sensor_msgs/msg/Imu` |
| `ignition.msgs.Clock` | `rosgraph_msgs/msg/Clock` |
| `ignition.msgs.Model` | `sensor_msgs/msg/JointState` |
| `ignition.msgs.NavSat` | `sensor_msgs/msg/NavSatFix` |

### 3. Add to bridge in `sim.launch.py`

```python
arguments=[
    # existing entries ...
    '/new/topic@ros_type@ignition.msgs.SomeType',   # bidirectional
    '/another/topic@ros_type[ignition.msgs.SomeType', # gz → ROS 2 only
],
```

### 4. Rebuild and relaunch

```bash
colcon build --packages-select puzzlebot_gz
source install/setup.bash
ros2 launch puzzlebot_gz sim.launch.py
```

### 5. Verify

```bash
ros2 topic list | grep new
ros2 topic hz /new/topic
```
