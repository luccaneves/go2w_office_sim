# GO2W Office Sim - Integration & Debugging Tech Report

Date: 2026-03-05  
Repo: `go2w_office_sim`  
Scope: Gazebo (gz-sim) simulation + SLAM Toolbox + Nav2 + autonomous frontier exploration, with an emphasis on the real integration issues encountered and how they were resolved.

## 1. Background

This project integrates a GO2W-like 4-wheel skid-steer robot model in Gazebo with:

- Sensor pipeline: GPU LiDAR + IMU
- Mapping: `slam_toolbox` (online async)
- Navigation: Nav2 (costmaps, planner, controller, collision monitor)
- Exploration: a custom BFS frontier explorer with GVD-based path planning (waypoints sampled every 0.5 m)

The main integration risk was not "adding one more node", but ensuring the **data contracts** between modules stayed consistent:

- TF frames exist and are connected (`map -> odom -> base -> lidar`)
- Topics are named consistently on the Gazebo transport side and the ROS 2 side
- LiDAR data is free of self-occlusion artifacts so Nav2 costmaps do not mark the robot as being inside an obstacle
- Odometry is stable enough for SLAM scan matching and for Nav2 to generate consistent plans

## 2. Key Commits (Debugging Timeline)

- `d8369db`: Fix lateral slip artifacts by disabling wheel-encoder odom from DiffDrive and publishing ground-truth odom from physics.
- `6bb5a95`: Reduce SLAM "rotation update" threshold; strengthen wheel-ground contact; increase LiDAR min range.
- `8bebe21`: Integrate Nav2 + exploration; introduce `/scan_raw -> box filter -> /scan` pipeline; unify Gazebo/ROS topic naming for LiDAR.

## 3. System Overview (What Must Stay Consistent)

TF contract (expected):

- `map` (from SLAM Toolbox)
- `odom` (from OdometryPublisher / physics)
- `base` (robot root)
- `lidar` (sensor frame)

Scan contract (expected):

- Gazebo LiDAR publishes `/scan_raw`
- `ros_gz_bridge` bridges `/scan_raw` into ROS 2
- `laser_filters` runs `scan_to_scan_filter_chain`: `/scan_raw` -> `/scan`
- Both SLAM Toolbox and Nav2 consume `/scan`

Odom contract (expected):

- DiffDrive drives wheel joints from `/cmd_vel` but does not publish odom/TF
- OdometryPublisher publishes `/odom` and TF `odom->base` at 50 Hz (2D)

## 4. Errors and Fixes

### 4.1 Wheel Slip During Straight Driving Causing SLAM Overlapping

**Symptom**

- SLAM map overlapped (double walls / duplicated geometry), especially noticeable when driving straight or turning.
- Odom-based rotation could be significantly wrong for a skid-steer model (e.g., 90 deg actual motion reported as ~180 deg).

**Root cause**

- A 4-wheel skid-steer inherently causes lateral scrub during turning.
- Using DiffDrive wheel encoder odometry in this setup can over-estimate yaw, corrupting SLAM submaps.
- On top of that, the simulated wheel-ground contact was not firm enough, allowing unrealistic slip.

**Fix (A): Use ground-truth odometry from physics (primary fix)**

- Commit: `d8369db`
- File: `urdf/go2w_gz.urdf.xacro`
- Change:
  - DiffDrive plugin continues to drive wheels from `/cmd_vel`, but stops publishing odom/TF (`odom_publish_frequency` set to `0`, publishing to `/odom_unused` and `/tf_unused`).
  - `gz::sim::systems::OdometryPublisher` is enabled to publish `/odom` and `odom->base` TF at 50 Hz from the model pose (2D mode).

**Fix (B): Make wheels "grab" the ground more firmly (physics tuning)**

- Commit: `6bb5a95`
- File: `urdf/go2w_gz.urdf.xacro`
- Change (wheel contact):
  - `mu1/mu2: 1.5 -> 100`
  - add `kp=1e6`, `kd=100`, `min_depth=0.001`

**Verification**

- Drive with keyboard teleop and watch SLAM:
  - Map no longer overlaps badly during turns/straight driving.
  - `/odom` yaw behaves consistently with the simulated pose.

### 4.2 SLAM Not Updating During Rotation

**Symptom**

- Rotating in place did not reliably create new map updates; map updates happened mainly after translating a minimum distance.

**Root cause**

- `slam_toolbox` update gating was too strict on heading change.

**Fix**

- Commit: `6bb5a95`
- File: `config/slam_params.yaml`
- Change:
  - `minimum_travel_heading: 0.3 -> 0.05` rad (about 3 deg)

**Verification**

- Rotate in place and observe that scan matching / map updates now trigger on smaller heading changes.

### 4.3 LiDAR Self-Occlusion Creating False Obstacles Behind Robot

**Symptom**

- RViz showed occupied cells behind the robot that moved with the robot.
- Nav2 costmaps marked obstacles near/inside the robot footprint, leading to planning failures.

**Root cause**

- LiDAR rays hit the robot body and rear hips/legs behind the sensor.
  - Body self-hit distance: ~0.1 m
  - Hips/legs self-hit distance: ~0.5 m

**Attempt 1 (insufficient): Increase LiDAR min range**

- Commit: `6bb5a95`
- File: `urdf/go2w_gz.urdf.xacro`
- Change:
  - LiDAR `range.min: 0.1 -> 0.3`
- Why it was insufficient:
  - It filtered the body hits at ~0.1 m, but **did not remove** leg/hip hits at ~0.5 m.

**Final fix (surgical): LaserScanBoxFilter for self-hit removal**

- Commit: `8bebe21`
- File: `config/laser_filter.yaml`
- Implementation:
  - Install/use `laser_filters`.
  - Add `laser_filters/LaserScanBoxFilter` in the `lidar` frame to remove points in a box behind the sensor:
    - `x: [-0.55, 0]`, `y: [-0.22, 0.22]`, `z: [-1, 1]`
    - `invert: false` (remove points inside the box)
- Rationale:
  - Removes self-hits without "blinding" the robot to close walls in front.
  - Points behind the sensor are physically occluded by the body anyway, so this is typically safe.

**Verification**

- Compare `/scan_raw` vs `/scan` in RViz:
  - `/scan` no longer contains the self-hit cluster behind the robot.
  - Costmaps stop placing a persistent obstacle at the robot start position.

### 4.4 Nav2 Nodes Not Running When User Tried 2D Nav Goal

**Symptom**

- Clicking "2D Nav Goal" in RViz did nothing (no planning, no motion).

**Root cause**

- The user launched `slam_test.launch.py`, which intentionally runs SLAM only (no Nav2 stack).

**Fix**

- Use the Nav2 launch:
  - `launch/nav2_office.launch.py` for manual navigation
  - `launch/auto_explore.launch.py` for full exploration stack

**Verification**

- Confirm Nav2 nodes exist (`planner_server`, `controller_server`, `bt_navigator`) and lifecycle manager autostarts them.

### 4.5 "Start occupied" Planner Error

**Symptom**

- Planner warning (Nav2 GridBased plugin): `'Start occupied'`

**Root cause**

- Costmap marked obstacles inside/near the robot footprint due to LiDAR self-occlusion.

**First fix attempt (mitigation only)**

- Commit: `8bebe21`
- File: `config/nav2_params.yaml`
- Changes:
  - Shrink global costmap `robot_radius: 0.18 -> 0.10`
  - Raise costmap obstacle layer `obstacle_min_range: 0.0 -> 0.3` (local + global)
- Outcome:
  - Helped reduce near-field noise, but did **not** address the main self-hit cluster at ~0.5 m.

**Proper fix**

- A key debugging clue from the user was: "the self-occlusion problem hasn't solved yet and that's why it reports start occupied".
- Same root cause as Section 4.3:
  - Implement the LiDAR box filter to remove self-hits before Nav2 consumes the scan.

### 4.6 "Invalid frame ID 'map'" - SLAM Not Getting Scan Data

**Symptom**

- Nav2 costmaps timed out waiting for TF from `base` to `map`:
  - `Invalid frame ID 'map' ... frame does not exist`

**Root cause**

- SLAM never published `map` because it received no scan data.
- After switching to a filtered pipeline, the bridge expected `/scan_raw`, but Gazebo was still publishing LiDAR to `/scan`.
  - `ros_gz_bridge` uses the same topic name on the Gazebo and ROS sides for `parameter_bridge`, so a name mismatch results in "silent no data".

**Fix**

- Commit: `8bebe21`
- File: `urdf/go2w_gz.urdf.xacro`
- Change:
  - LiDAR sensor topic: `<topic>/scan</topic>` -> `<topic>/scan_raw</topic>`
- Launch integration:
  - Bridge `/scan_raw` from Gazebo to ROS 2.
  - Run filter chain: `/scan_raw` -> `/scan`.
  - Configure SLAM Toolbox and Nav2 to consume `/scan`.

**Verification**

- `ros2 topic echo /scan_raw --once` returns data.
- `ros2 topic echo /scan --once` returns filtered data.
- `ros2 run tf2_ros tf2_echo map base` succeeds after SLAM initializes.

## 5. Problem Solving

- SLAM map overlapping during straight driving: traced to wheel slip / bad wheel odom -> increased friction coefficients + switched to ground-truth odom.
- SLAM not updating on rotation: reduced `minimum_travel_heading` threshold.
- Persistent "Start occupied": first tried costmap tuning (robot radius, obstacle min range), then identified root cause as LiDAR self-occlusion (legs/hips ~0.5 m) -> added `laser_filters` box filter as a surgical fix.
- Scan pipeline broken after adding filter: Gazebo/ROS topic mismatch on `/scan_raw` -> aligned LiDAR `<topic>` in URDF.

What consistently worked during debugging:

- Start from observables:
  - TF existence and connectivity (`map`, `odom`, `base`, `lidar`)
  - Raw topics (`/scan_raw`, `/odom`) before blaming downstream consumers
- Prefer root-cause fixes over parameter "hacks":
  - Costmap tuning can mitigate symptoms, but sensor self-hits must be removed at the source (or in a filter) to avoid systemic failures.
- Make data pipelines explicit:
  - Once `/scan_raw -> filter -> /scan` was standardized, SLAM/Nav2 behavior became predictable.

## 6. All User Messages (Verbatim)

- "1. it's better now, but when robot goes straight, the wheels also slip and causing overlapping,can we make the wheels grab the ground really firmly bcs it's true in reality. now i can only drive it slow to prevent mapping degeneration. can you come up with other brilliant ideas? 2. when rotates for certain angle, you also need to set a map point, not only for moving certain distance, due to the bad odom. 3. there's always a occlusion back of the lidar so that there's always obs grid hehind the robot when slam in map point, filter out the lidar points within the robot radius"
- "good, now let's get into next stage: join the nav2 part into the process(actually i had one before). i will give robot a nav goal in rviz. and nav2 structure should take auto_explore_sim/config/nav2_params.yaml as strong reference."
- "when i set 2d goal pose in rviz, it didn't work"
- "next, you need to implement the frontier explorer based on auto_explore_sim/launch/auto_explore.launch.py and auto_explore_sim/scripts/frontier_explorer.py work."
- "but it won't work: [planner_server-8] [WARN] [1772671921.342104591] [planner_server]: GridBased plugin failed to plan from (-0.59, 0.04) to (3.99, -0.71): 'Start occupied' ... what issue might be"
- "now you should take closer look at auto_explore_sim/launch/auto_explore.launch.py, rviz shows GVD path and candidate frontiers. and path planning is based on the voronoi map for each 50cm setting a goal, right? then, the self-occlusion problem hasn't solved yet and that's why it reports start occupied."
- "[planner_server-9] [INFO] [1772673216.328087781] [global_costmap.global_costmap]: Timed out waiting for transform from base to map to become available, tf error: Invalid frame ID 'map' passed to canTransform argument target_frame - frame does not exist"
- "write a readme.md for this go2w_office_sim, explicit the architexture and struture, how modules cooperate together(communicate, topics etc.)"

## 7. Notes / Caveats

- `launch/office_sim.launch.py` bridges `/scan` (legacy pipeline). The main SLAM/Nav2 launches use `/scan_raw` + filtering.
  - If you need LiDAR in `office_sim.launch.py`, align it with `/scan_raw` or use `slam_test.launch.py`.
