# Duaro (WD002N) ROS2 Jazzy + Isaac Sim Integration

Dual-arm SCARA robot simulation using Isaac Sim with ROS2 Jazzy, web streaming, and MoveIt2 motion planning.

## Architecture

```
┌─────────────────────────────────┐     ROS2 Topics      ┌──────────────────────────────────┐
│        Isaac Sim (Py 3.11)      │◄────────────────────►│        ROS2 Side (Py 3.12)       │
│                                 │                       │                                  │
│  isaac_sim_duaro.py             │  /joint_states_raw ──►│  joint_state_reorder_duaro.py    │
│  - Physics simulation           │  /tf ────────────────►│    └──► /joint_states             │
│  - URDF import + drives         │  /clock ─────────────►│                                  │
│  - Prismatic J3 support         │  /joint_commands ◄────│  trajectory_bridge_duaro.py      │
│  - OmniGraph ROS2 bridge        │                       │    └──► FollowJointTrajectory     │
│  - Web streaming (livestream)   │                       │                                  │
│                                 │                       │  MoveIt move_group               │
│                                 │                       │    └──► lower_arm / upper_arm     │
│                                 │                       │                                  │
│                                 │                       │  robot_state_publisher            │
└─────────────────────────────────┘                       └──────────────────────────────────┘
```

## Package Structure

```
duaro_ws/
  src/duaro_description/
    urdf/duaro.urdf.xacro          # WD002N URDF (ported from ROS1 khi_duaro_description)
    meshes/                        # 18 WD002N STL mesh files
    config/                        # Joint limits, MoveIt configs, kinematics
    srdf/duaro.srdf                # Semantic robot description (planning groups, collisions)
    launch/duaro_isaac_ros.launch.py  # ROS2 side launch
    scripts/
      isaac_sim_duaro.py           # Isaac Sim standalone script
      joint_state_reorder_duaro.py # Joint state relay
      trajectory_bridge_duaro.py   # Trajectory action -> topic bridge
```

## Setup

### 1. Build the workspace

```bash
cd /root/duaro_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 2. Generate the flat URDF for Isaac Sim (one-time)

```bash
cd /root/duaro_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
xacro $(ros2 pkg prefix duaro_description)/share/duaro_description/urdf/duaro.urdf.xacro \
  | sed "s|package://duaro_description/|$(ros2 pkg prefix duaro_description)/share/duaro_description/|g" \
  > /root/duaro_ws/duaro_isaac.urdf
```

## Running

### Terminal 1: Start Isaac Sim

```bash
/isaac-sim/python.sh /root/duaro_ws/src/duaro_description/scripts/isaac_sim_duaro.py
```

Wait for: `Duaro (WD002N) simulation running.`

Connect to the web streaming URL printed to the console.

### Terminal 2: Start ROS2 side

```bash
cd /root/duaro_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch duaro_description duaro_isaac_ros.launch.py
```

With RViz (via VNC):
```bash
ros2 launch duaro_description duaro_isaac_ros.launch.py use_rviz:=true
```

RViz via noVNC: `http://localhost:6080/vnc.html?autoconnect=true&password=password`

### Terminal 3: Verify and test

```bash
source /opt/ros/jazzy/setup.bash

# Check topics
ros2 topic list
# Expected: /joint_states, /joint_states_raw, /tf, /clock, /joint_commands

# Check joint states (8 joints)
ros2 topic echo /joint_states --once

# Check action servers
ros2 action list
# Expected:
#   /duaro_lower_arm_controller/follow_joint_trajectory
#   /duaro_upper_arm_controller/follow_joint_trajectory
```

## Testing Motion

### Move the lower arm

```bash
ros2 action send_goal /duaro_lower_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [lower_joint1, lower_joint2, lower_joint3, lower_joint4],
    points: [
      {positions: [-1.57, 0.78, 0.05, 0.0], time_from_start: {sec: 3}}
    ]
  }}"
```

### Move the upper arm

```bash
ros2 action send_goal /duaro_upper_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [upper_joint1, upper_joint2, upper_joint3, upper_joint4],
    points: [
      {positions: [1.57, -0.78, 0.12, 0.0], time_from_start: {sec: 3}}
    ]
  }}"
```

## Robot Details

| Property | Value |
|----------|-------|
| Model | WD002N (Duaro) |
| Type | Dual-arm SCARA |
| Arms | lower_arm (4 DOF), upper_arm (4 DOF) |
| Joint types | J1: revolute, J2: revolute, J3: **prismatic**, J4: revolute |
| Total active joints | 8 (6 revolute + 2 prismatic) |
| Home positions | lower: [-45deg, 45deg, 0.09m, 0deg] / upper: [45deg, -45deg, 0.09m, 0deg] |

### Joint Limits

| Joint | Type | Min | Max | Max Vel |
|-------|------|-----|-----|---------|
| lower_joint1 | revolute | -2.967 rad | 2.967 rad | 4.189 rad/s |
| lower_joint2 | revolute | -2.443 rad | 2.443 rad | 5.498 rad/s |
| lower_joint3 | **prismatic** | 0.0 m | 0.15 m | 0.255 m/s |
| lower_joint4 | revolute | -6.283 rad | 6.283 rad | 10.472 rad/s |
| upper_joint1 | revolute | -2.443 rad | 8.727 rad | 4.189 rad/s |
| upper_joint2 | revolute | -2.443 rad | 2.443 rad | 5.498 rad/s |
| upper_joint3 | **prismatic** | 0.0 m | 0.15 m | 0.255 m/s |
| upper_joint4 | revolute | -6.283 rad | 6.283 rad | 10.472 rad/s |

## Troubleshooting

- **URDF import fails**: Regenerate the flat URDF (step 2 in Setup). Check mesh STLs exist in the installed `meshes/` dir.
- **No /joint_states topic**: Ensure Isaac Sim is publishing `/joint_states_raw` and the reorder node is running.
- **MoveIt can't find controllers**: Ensure `trajectory_bridge_duaro.py` is running.
- **Robot falls through floor**: The URDF must have `fix_base = True` in the Isaac Sim import config.
- **RViz crashes (SIGABRT)**: dbus issue in containers. The launch file sets `DBUS_SESSION_BUS_ADDRESS=/dev/null`.
