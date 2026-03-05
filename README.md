# GO2W Office Simulation

Unitree GO2W quadruped robot simulation in an office environment with autonomous exploration capabilities.

Built on Gazebo (gz-sim), ROS 2 Jazzy, SLAM Toolbox, Nav2, and a custom BFS frontier explorer with GVD (Generalized Voronoi Diagram) path planning.

## Package Structure

```
go2w_office_sim/
├── config/
│   ├── frontier_explorer_params.yaml   # Frontier explorer tuning
│   ├── laser_filter.yaml               # Self-occlusion box filter
│   ├── nav2_params.yaml                # Nav2 stack (MPPI, SmacPlanner2D, costmaps)
│   └── slam_params.yaml               # SLAM Toolbox online_async
├── launch/
│   ├── office_sim.launch.py            # Minimal: Gazebo + robot + bridge (teleop only)
│   ├── slam_test.launch.py             # + SLAM Toolbox (no Nav2, for mapping tests)
│   ├── nav2_office.launch.py           # + Nav2 (manual 2D Nav Goal in RViz)
│   └── auto_explore.launch.py          # + Frontier Explorer (full autonomous)
├── scripts/
│   └── frontier_explorer.py            # BFS frontier search + GVD path planning
├── urdf/
│   ├── go2w_gz.urdf.xacro              # Robot model + Gazebo plugins
│   └── const.xacro                     # Physical constants (dimensions, inertia)
├── meshes/                             # Visual meshes for robot links
├── worlds/
│   └── office.sdf                      # Office environment
└── rviz/
    ├── nav2.rviz                       # Nav2 visualization
    └── explore.rviz                    # Exploration (frontiers, GVD, paths)
```

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Gazebo (gz-sim)                              │
│  ┌──────────┐  ┌──────────────────┐  ┌───────────────────────────┐ │
│  │  Office   │  │   GO2W Robot     │  │      Gazebo Plugins       │ │
│  │  World    │  │  (DiffDrive      │  │  - OdometryPublisher      │ │
│  │ (SDF)     │  │   4-wheel)       │  │  - JointStatePublisher    │ │
│  └──────────┘  └──────────────────┘  │  - GPU LiDAR sensor       │ │
│                                       │  - IMU sensor              │ │
│                                       └───────────────────────────┘ │
└────────────────────────────┬────────────────────────────────────────┘
                             │ ros_gz_bridge
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Layer                                 │
│                                                                     │
│  /scan_raw ──► laser_filters ──► /scan ──┬──► SLAM Toolbox ──► /map│
│                (box filter)              │                          │
│                                          ├──► Nav2 Costmaps         │
│                                          └──► Collision Monitor     │
│                                                                     │
│  /map ──► Frontier Explorer ──► Nav2 Actions ──► cmd_vel pipeline  │
│                                                                     │
│  Planner ──► Controller ──► Vel Smoother ──► Collision Mon ──► Robot│
│  (SmacPlanner2D) (MPPI)    /cmd_vel_nav    /cmd_vel_smoothed /cmd_vel│
└─────────────────────────────────────────────────────────────────────┘
```

### TF Tree

```
map                          (published by: SLAM Toolbox)
 └── odom                    (published by: gz-sim OdometryPublisher, 50 Hz)
      └── base               (robot root)
           └── body          (published by: robot_state_publisher)
                ├── lidar
                ├── imu
                ├── camera
                ├── Head_upper → Head_lower
                ├── FL_hip → FL_thigh → FL_calf → FL_foot
                ├── FR_hip → FR_thigh → FR_calf → FR_foot
                ├── RL_hip → RL_thigh → RL_calf → RL_foot
                └── RR_hip → RR_thigh → RR_calf → RR_foot
```

### Scan Pipeline

The raw lidar hits the robot's own body/legs behind the sensor. A `laser_filters` box filter removes these self-occlusion returns before any consumer sees them.

```
Gazebo GPU LiDAR (360 samples, 0.3-20m range, 10 Hz)
       │
       ▼
  /scan_raw ──► ros_gz_bridge ──► /scan_raw (ROS 2)
                                      │
                                      ▼
                              laser_filters node
                              (LaserScanBoxFilter)
                              removes points in box:
                              x: [-0.55, 0], y: [-0.22, 0.22]
                              (robot body behind lidar)
                                      │
                                      ▼
                                    /scan ──┬──► SLAM Toolbox
                                            ├──► Local Costmap
                                            ├──► Global Costmap
                                            └──► Collision Monitor
```

### cmd_vel Pipeline

Nav2 uses a multi-stage velocity pipeline with safety guarantees:

```
                    ┌──────────────────┐
                    │  BT Navigator    │  (behavior tree: replanning + recovery)
                    └────────┬─────────┘
                             ▼
                    ┌──────────────────┐
                    │  Planner Server  │  SmacPlanner2D on global costmap
                    │  → /plan         │  A* search, allow_unknown=true
                    └────────┬─────────┘
                             ▼
                    ┌──────────────────┐
                    │ Controller Server│  MPPI (1000 trajectories, DiffDrive)
                    │ → /cmd_vel_nav   │  vx_max=1.0 m/s, wz_max=1.9 rad/s
                    └────────┬─────────┘
                             ▼
                    ┌──────────────────┐
                    │ Velocity Smoother│  Accel limits: 1.5 m/s², 2.5 rad/s²
                    │→ /cmd_vel_smoothed│  Prevents jerky motion
                    └────────┬─────────┘
                             ▼
                    ┌──────────────────┐
                    │Collision Monitor │  FootprintApproach: slows near obstacles
                    │  → /cmd_vel      │  Uses /scan directly (fastest reaction)
                    └────────┬─────────┘
                             ▼
                    ┌──────────────────┐
                    │  DiffDrive       │  Gazebo plugin drives 4 wheels
                    │  (gz-sim)        │  wheel_sep=0.384m, wheel_r=0.085m
                    └──────────────────┘
```

### Frontier Explorer

Autonomous exploration using BFS frontier search with GVD-based path planning.

```
                         ┌─────────┐
                    ┌────│  /map   │◄──── SLAM Toolbox
                    │    └─────────┘
                    ▼
           ┌────────────────────┐
           │  Frontier Explorer │
           │                    │  1. BFS from robot position
           │  Subscriptions:    │  2. Find frontier clusters (unknown/free boundary)
           │   /map             │  3. Cost = min_dist - clearance_scale * GVD_clearance
           │   /explore/resume  │  4. Compute GVD path (A* on Voronoi skeleton)
           │                    │  5. Sample waypoints every 0.5m
           │  Publications:     │  6. Send NavigateThroughPoses to Nav2
           │   /explore/frontiers│  7. Blacklist on failure, replan on success
           │   /explore/gvd     │
           │   /explore/gvd_path│
           └────────┬───────────┘
                    │ Nav2 Action Clients
                    ▼
           navigate_to_pose          (single goal, fallback)
           navigate_through_poses    (GVD waypoints, preferred)
```

**Selection algorithm:**
- BFS outward from robot to find frontier cells (unknown cells adjacent to free space)
- Cluster into frontiers, filter by `min_frontier_size` (20 cells)
- Cost = `min_distance` - `clearance_scale` * GVD_clearance (prefer near + wide corridors)
- A* on GVD skeleton finds corridor-center paths; waypoints sampled every 0.5m
- Progress timeout (30s) and blacklisting (120s) prevent stuck loops

### Odometry Strategy

The robot uses **ground-truth odometry** from the physics engine instead of wheel-based odometry:

```
DiffDrive plugin:
  - Drives wheels from /cmd_vel          ✓ (motor control)
  - Publishes odom to /odom_unused       ✗ (suppressed, inaccurate due to skid-steer)
  - Publishes TF to /tf_unused           ✗ (suppressed)

OdometryPublisher plugin:
  - Reads model pose from physics engine  ✓ (ground truth, no wheel slip)
  - Publishes to /odom @ 50 Hz           ✓
  - Publishes odom→base TF               ✓
  - 2D mode (x, y, yaw only)             ✓
```

**Why:** 4-wheel skid-steer causes lateral wheel scrub during turns. DiffDrive's wheel encoder odometry over-reports rotation (e.g., 90 actual becomes ~180 reported), which corrupts SLAM submaps and causes map overlapping.

## Launch Files

| Launch File | Components | Use Case |
|---|---|---|
| `office_sim.launch.py` | Gazebo + Robot + Bridge | Basic teleop testing |
| `slam_test.launch.py` | + Scan Filter + SLAM + RViz | Mapping quality testing |
| `nav2_office.launch.py` | + Nav2 Stack | Manual navigation (2D Nav Goal) |
| `auto_explore.launch.py` | + Frontier Explorer (15s delay) | Full autonomous exploration |

## Usage

### Autonomous Exploration
```bash
ros2 launch go2w_office_sim auto_explore.launch.py
```

Frontier explorer starts after 15 seconds. Control:
```bash
# Pause exploration
ros2 topic pub /explore/resume std_msgs/Bool '{data: false}' --once

# Resume exploration
ros2 topic pub /explore/resume std_msgs/Bool '{data: true}' --once
```

### Manual Navigation
```bash
ros2 launch go2w_office_sim nav2_office.launch.py
```
Use **2D Nav Goal** in RViz to set navigation goals.

### SLAM Testing
```bash
ros2 launch go2w_office_sim slam_test.launch.py

# In another terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Key Parameters

### Robot Physical Properties
| Property | Value |
|---|---|
| Wheel separation | 0.384 m |
| Wheel radius | 0.085 m |
| Robot radius (local costmap) | 0.25 m |
| Robot radius (global costmap) | 0.10 m (relaxed to avoid "Start occupied") |
| Wheel friction (mu1/mu2) | 100 (rubber-on-ground, no-slip) |

### MPPI Controller Critics
| Critic | Weight | Purpose |
|---|---|---|
| ConstraintCritic | 4.0 | Kinematic constraint violations |
| CostCritic | 5.0 | Costmap obstacle avoidance |
| GoalCritic | 5.0 | Distance to goal (near goal) |
| GoalAngleCritic | 3.0 | Heading at goal |
| PreferForwardCritic | 15.0 | Strongly prefer forward motion |
| PathAlignCritic | 22.0 | Lateral deviation from path |
| PathFollowCritic | 12.0 | Distance to furthest path point |
| PathAngleCritic | 15.0 | Heading deviation from path |

## Dependencies

- `ros_gz_sim`, `ros_gz_bridge` - Gazebo simulation
- `robot_state_publisher`, `xacro` - Robot description
- `slam_toolbox` - Online async SLAM
- `nav2_bringup`, `nav2_common` - Navigation stack
- `laser_filters` - Scan self-occlusion filtering
- `rviz2` - Visualization
- `teleop_twist_keyboard` - Manual driving
