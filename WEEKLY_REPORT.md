# Weekly Report: GO2W Office Sim Integration & Debugging

Week: 2026-03-02 to 2026-03-05 (Asia/Shanghai)  
Workspace: `~/ros2_ws/src`  

## 1. TL;DR

This week we turned a "GO2W model dropped into an existing exploration stack" into a **working, end-to-end** simulation package (`go2w_office_sim`) by fixing the real blockers: **odometry reliability**, **LiDAR self-occlusion**, and **topic/TF contracts**.

Comparison target:

- Working: `go2w_office_sim` (`master`, commit `8bebe21`)
- Non-working (AI-only attempt): `auto_explore_sim` branch `wrong_go2w_auto_sim` (commit `bb70787`)

## 2. What Works Now (Current Project)

`go2w_office_sim` runs a stable pipeline:

- Gazebo + GO2W robot spawn
- Bridge: `/cmd_vel`, `/odom`, `/tf`, `/scan_raw`, `/imu/data`, `/clock`
- Scan pipeline: `/scan_raw` -> `laser_filters` (box filter) -> `/scan`
- SLAM Toolbox consumes `/scan`, publishes `map` and `map->odom`
- Nav2 consumes `/scan` (costmaps + collision monitor)
- Frontier explorer sends goals via Nav2 actions

The key property is that the system contracts are explicit and consistent:

- TF: `map -> odom -> base -> lidar`
- Scan: `/scan_raw` exists, filtered `/scan` exists, and both SLAM/Nav2 subscribe to `/scan`
- Odom: `/odom` is stable (physics-ground-truth), not wheel-encoder-based for skid-steer

## 3. Why `wrong_go2w_auto_sim` Still Fails (Root Causes)

The `wrong_go2w_auto_sim` branch did add a GO2W URDF, a bridge config, and tried to tune Nav2/SLAM parameters. But the integration remains fragile because it does not address the two core failure sources:

### 3.1 Wheel-Encoder Odom From DiffDrive (Skid-Steer Artifact)

In `wrong_go2w_auto_sim`:

- DiffDrive publishes `/odom` + `/tf` directly (`odom_publish_frequency=30`).
- The robot is effectively skid-steer (4 wheels). During turns it scrubs laterally and wheel-encoder odom tends to **over-report yaw**, which corrupts SLAM and downstream planners.

In `go2w_office_sim`:

- DiffDrive is used only to drive wheel joints from `/cmd_vel` (odom publishing disabled).
- `/odom` + `odom->base` TF come from Gazebo `OdometryPublisher` (model pose, 2D), removing wheel-slip artifacts.

### 3.2 LiDAR Self-Occlusion Not Removed at the Source

In `wrong_go2w_auto_sim`:

- The scan is bridged directly to `/scan` and consumed as-is.
- It relies on a small `range.min` (0.2 m) and costmap `obstacle_min_range` (0.2 m) to "ignore near self-returns".
- This does not remove self-hits farther out (e.g., legs/hips around ~0.5 m in our GO2W model), so SLAM can still paint false obstacles into the map. Once the map is contaminated, the global planner still sees "start occupied".

In `go2w_office_sim`:

- We add `laser_filters/LaserScanBoxFilter` in the `lidar` frame to surgically remove points behind the sensor inside the robot envelope.
- Result: SLAM/Nav2 never see those self-hits; the map and costmaps stay clean around the robot.

### 3.3 Symptom Tuning vs Contract Fixes

`wrong_go2w_auto_sim` attempts several mitigations (smaller global `robot_radius`, min ranges, removing some layers). These can reduce the frequency of failures but cannot guarantee correctness if:

- odom is wrong (SLAM pose-graph anchors to a bad prior)
- scans contain robot self-hits (map/costmap "occupied at start")

Our working project fixed the upstream contracts first, then tuned parameters.

## 4. Side-by-Side Highlights (What’s Different)

| Area | `wrong_go2w_auto_sim` (`auto_explore_sim@bb70787`) | `go2w_office_sim` (`go2w_office_sim@8bebe21`) | Effect |
|---|---|---|---|
| Odom source | DiffDrive publishes odom/TF | OdometryPublisher publishes odom/TF; DiffDrive odom disabled | Removes skid-steer yaw artifacts |
| Wheel contact | `mu1/mu2=3.0` only | `mu1/mu2=100` + `kp/kd/min_depth` | Much less slip, more realistic traction |
| Scan topic | `scan` bridged directly to `/scan` | Gazebo publishes `/scan_raw`, filtered to `/scan` | Enables robust filtering + debugging |
| Self-occlusion handling | min range + costmap min range heuristics | `LaserScanBoxFilter` removes self-hits | Prevents map/costmap contamination |
| Planner "Start occupied" | mitigated via parameters, but root cause remains | root cause removed at scan stage | Planner becomes stable at start |

Concrete code evidence (most relevant deltas):

- `auto_explore_sim@bb70787:urdf/go2w_harmonic.urdf.xacro`
  - wheel contact: `<mu1>3.0</mu1>`, `<mu2>3.0</mu2>`
  - DiffDrive odom: `<odom_topic>odom</odom_topic>`, `<tf_topic>tf</tf_topic>`, `<odom_publish_frequency>30</odom_publish_frequency>`
  - LiDAR topic + min range: `<topic>scan</topic>`, `<min>0.2</min>`
- `go2w_office_sim@8bebe21:urdf/go2w_gz.urdf.xacro`
  - wheel contact: `<mu1>100</mu1>`, `<mu2>100</mu2>`, plus `kp/kd/min_depth`
  - DiffDrive odom disabled: `<odom_publish_frequency>0</odom_publish_frequency>` and publishes to `/odom_unused`
  - OdometryPublisher enabled: publishes `/odom` and `/tf` at 50 Hz (2D)
  - LiDAR publishes `/scan_raw` (then filtered to `/scan`)
- `go2w_office_sim@8bebe21:config/laser_filter.yaml`
  - `LaserScanBoxFilter` removes points in `x=[-0.55,0], y=[-0.22,0.22]` (lidar frame)

## 5. Step-by-Step Problem Solving (What We Actually Did)

### 2026-03-04: Baseline Bringup

1. Created a minimal GO2W simulation bringup (spawn + bridge + teleop).
   - `go2w_office_sim` commits: `707d181` (v1), `12f0b62` (v1_teleop)
2. Added SLAM Toolbox and baseline Nav2 structure (enough to reproduce real navigation failures).
   - `go2w_office_sim` commit: `d1daa03` (v2_slam_teleop)
3. Observed two dominant failure signatures early:
   - SLAM map overlap / deformation tied to motion and turning
   - Persistent occupied cells behind the robot from self-occlusion

### 2026-03-05: Root Causes, Fixes, and Verification

4. Fixed odometry reliability first:
   - Disabled DiffDrive odom publishing and switched to physics-ground-truth odometry (`OdometryPublisher`).
   - Increased wheel-ground traction (`mu1/mu2`, plus contact parameters).
   - `go2w_office_sim` commits: `d8369db`, `6bb5a95`
5. Made SLAM update during rotation:
   - Reduced `minimum_travel_heading` so SLAM adds keyframes on small yaw changes.
   - `go2w_office_sim` commit: `6bb5a95`
6. Tried the "cheap" LiDAR fix and proved it insufficient:
   - Increasing `range.min` only removes very near body hits; leg/hip hits remained.
7. Implemented the correct self-occlusion fix:
   - Added a `laser_filters` box filter in the `lidar` frame to remove points behind the sensor within the robot envelope.
   - `go2w_office_sim` commit: `8bebe21`
8. Repaired a scan pipeline integration pitfall:
   - Ensured Gazebo LiDAR topic naming and bridge topic naming match (otherwise SLAM receives no scans and `map` frame never appears).
   - `go2w_office_sim` commit: `8bebe21` (LiDAR `<topic>` aligned to `/scan_raw`)
9. Final integration:
   - Nav2 + frontier exploration launch runs end-to-end with stable planning and mapping.
   - `go2w_office_sim` commit: `8bebe21`

## 6. Lessons Learned (Why the Working Version Looks “More Boring”)

- Robotics stacks fail at the boundaries: TF, topic naming, sensor artifacts, and odom assumptions.
- Parameter tuning can hide symptoms, but it cannot replace:
  - stable odometry
  - clean scans
  - explicit, verifiable data contracts

## 7. Nav2 Controller Tuning (2026-03-06)

### 7.1 Problem: Robot Collisions & Poor Turning

The initial Nav2 config had aggressive speed limits (`vx_max: 1.0`), weak obstacle avoidance (`CostCritic: 5.0`, thin inflation), and MPPI critic weights that fought each other during turns. The robot collided with obstacles and couldn't rotate when the path was perpendicular to its heading.

### 7.2 Root Cause: MPPI Critic Deadlock on DiffDrive

When the planned path is perpendicular (90°) to the robot's heading, MPPI enters a deadlock:

- `PathAlignCritic` (high weight) wants the robot to move sideways onto the path — impossible for DiffDrive
- `PathAngleCritic` (low weight) wants rotation — but is outweighed
- All sampled trajectories score poorly → near-zero velocity output → robot stops

### 7.3 Solution: RotationShimController + Balanced Critics

**RotationShimController** wraps MPPI to handle large heading errors:

```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "nav2_mppi_controller::MPPIController"
  angular_dist_threshold: 0.20   # rotate in-place until within ~11° of path
  rotate_to_heading_angular_vel: 1.8  # rad/s
  max_angular_accel: 3.2
```

When heading error > 11°, the shim rotates the robot in-place before handing off to MPPI. This breaks the deadlock cleanly.

**Note:** `FeasiblePathHandler` (with `enforce_path_rotation`) is a newer Nav2 alternative but is unavailable on Humble. The RotationShimController works on Humble/Iron/Jazzy.

### 7.4 MPPI Tuning Summary (Official Nav2 Defaults as Reference)

Key changes from initial config → tuned config (referencing `nav2_params_official.yaml`):

| Parameter | Initial | Tuned | Rationale |
|-----------|---------|-------|-----------|
| `vx_max` / `vx_min` | 1.0 / -1.0 | **0.5 / -0.5** | Safer office speed; less collision risk |
| `wz_max` | 2.5 | **1.9** | Matches official default |
| `wz_std` | 1.2 | **0.8** | Less wobble; 0.4 (official) was too low for our setup |
| `iteration_count` | 2 | **1** | Sufficient with `regenerate_noises: true` |
| `regenerate_noises` | false | **true** | Better trajectory diversity per cycle |
| `PathAlignCritic.cost_weight` | 22.0 | **14.0** | Was dominating; caused deadlock with DiffDrive |
| `PathAlignCritic.max_path_occupancy_ratio` | 0.40 | **0.05** | Disables critic near obstacles (lets robot deviate to avoid) |
| `PathFollowCritic.cost_weight` | 12.0 | **5.0** | Less corner overshooting |
| `PathFollowCritic.offset_from_furthest` | 20 | **5** | Shorter lookahead carrot |
| `PathAngleCritic.threshold_to_consider` | 1.0 | **0.5** | Active closer to goal |
| `PreferForwardCritic` | disabled | **enabled (weight 5)** | Prevent unnecessary reversing |
| Local `inflation_radius` | 0.45 | **0.70** | Match official; better clearance |
| Global `inflation_radius` | 0.25 | **0.70** | Was barely robot radius; planner couldn't find safe paths |
| `cost_scaling_factor` (both) | 4-5 | **3.0** | Gentler falloff; wider safe zone |
| Velocity smoother `min_velocity[0]` | -0.10 | **-0.5** | Was bottlenecking reverse despite `vx_min: -0.5` |
| Velocity smoother `max_accel` | [1.5, 0, 2.5] | **[2.5, 0, 3.2]** | Match MPPI accel limits; prevent trajectory mismatch |
| Collision monitor `time_before_collision` | 0.5 | **1.2** | At 0.5 m/s, 0.5s = only 0.25m braking; 1.2s = 0.6m |

### 7.5 Costmap Inflation Tuning Guide

Inflation creates a "danger gradient" around obstacles. Two parameters control it:

- `inflation_radius`: how far the gradient extends (meters)
- `cost_scaling_factor`: how steeply cost drops with distance (higher = drops faster)

```
Wall   |████|▓▓▓|▒▒▒|░░░|   |       inflation_radius = 0.70
       lethal  inscribed  free
                ←── cost_scaling_factor controls decay rate ──→
```

- **Thin inflation (0.25m, factor 5)**: Planner/controller see very narrow buffers → paths hug walls → collisions
- **Thick inflation (0.70m, factor 3)**: Wider buffers → paths stay centered in corridors → safer but may fail in very tight spaces

### 7.6 TF Timing Issue: Local Costmap `global_frame`

The `odom→base` TF from Gazebo's `OdometryPublisher` via `ros_gz_bridge` has a massive timestamp delay (~1.77e9 seconds). This causes the local costmap's obstacle layer to silently drop all scan data when using `global_frame: odom`.

**Fix:** Set `local_costmap.global_frame: map` (not `odom`). This uses SLAM's correctly-timestamped `map→odom` TF path. Both local and global costmaps now use the `map` frame.

**Startup timing:** Since `map` frame doesn't exist until SLAM publishes its first TF, Nav2 nodes are wrapped in `TimerAction(period=10.0)` in the launch files to avoid "Invalid frame ID 'map'" errors.

### 7.7 Key Takeaways

1. **RotationShimController is essential for DiffDrive + MPPI** — without it, MPPI deadlocks when the path is perpendicular to the robot heading.
2. **Critic weight balance matters more than absolute values** — `PathAlignCritic >> CostCritic` means the robot hugs the path into obstacles. Keep obstacle avoidance critics competitive.
3. **Velocity smoother must match MPPI limits** — mismatched accel/velocity limits create trajectories the smoother can't execute, causing jerky or stalled motion.
4. **Inflation radius is the #1 safety knob** — thin inflation (< robot radius) guarantees collisions. Start with 2-3x robot radius.
5. **See `docs/mppi_tuning_handbook.md`** for the full parameter reference and common scenario fixes.

## 8. Next Steps

- Back-port the two hard requirements into any future integration branch:
  - physics-ground-truth odom (or a properly modeled skid-steer odom source)
  - scan self-occlusion filtering (box filter or equivalent)
- Align legacy launch files so all bringups use the same `/scan_raw -> /scan` contract.
- Test navigation in tighter environments and tune `CostCritic.consider_footprint: true` if collisions persist.
- Evaluate `SmacPlannerHybrid` (Hybrid A*) as an alternative to `SmacPlanner2D` for heading-aware path planning.
