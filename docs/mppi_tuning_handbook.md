# MPPI Controller Tuning Handbook for GO2W

Quick reference for fine-tuning `nav2_mppi_controller::MPPIController` FollowPath parameters.

---

## 1. Trajectory Sampling

These control HOW the controller explores possible trajectories.

| Parameter | Current | Range | Effect |
|-----------|---------|-------|--------|
| `batch_size` | 2000 | 500-5000 | Number of trajectory rollouts per cycle. More = better solutions, more CPU. |
| `time_steps` | 56 | 20-100 | How far ahead each trajectory looks (steps x model_dt = horizon). |
| `model_dt` | 0.05 | 0.02-0.1 | Time resolution per step. Current horizon = 56 x 0.05 = 2.8s. |
| `iteration_count` | 2 | 1-5 | Re-optimization passes per cycle. More = refined but slower. |
| `temperature` | 0.3 | 0.01-1.0 | Low = greedy (picks best), high = diverse (more exploration). |
| `regenerate_noises` | false | bool | true = fresh random samples each iteration. Better diversity, slightly more CPU. |

**Tuning tips:**
- Robot seems indecisive or oscillates? Increase `batch_size` or `iteration_count`.
- Robot doesn't find tight maneuvers? Increase `batch_size` to 3000+.
- Too slow on compute? Reduce `batch_size` or `time_steps`.
- Horizon too short to see upcoming turns? Increase `time_steps` or `model_dt`.

---

## 2. Velocity Limits

Hard constraints on what the robot can do.

| Parameter | Current | Description |
|-----------|---------|-------------|
| `vx_max` | 1.0 | Max forward speed (m/s) |
| `vx_min` | -1.0 | Max reverse speed (m/s). Negative = backward. |
| `vy_max` | 0.0 | Max lateral speed. 0 for DiffDrive (no sideways). |
| `wz_max` | 1.9 | Max angular velocity (rad/s). Higher = faster turns. |

**Tuning tips:**
- Robot too fast in tight spaces? Reduce `vx_max` (0.3-0.5 for offices).
- Robot won't back up? Make `vx_min` more negative (e.g., -0.5 or -1.0).
- Robot turns too slowly? Increase `wz_max`.
- Equal forward/backward: set `|vx_min| == vx_max`.

---

## 3. Noise Standard Deviations

Control the DIVERSITY of sampled trajectories. This is often the most impactful tuning.

| Parameter | Current | Range | Effect |
|-----------|---------|-------|--------|
| `vx_std` | 0.1 | 0.05-0.5 | Spread of forward velocity samples |
| `vy_std` | 0.1 | N/A | Irrelevant for DiffDrive (vy_max=0) |
| `wz_std` | 0.8 | 0.1-1.5 | Spread of angular velocity samples |

**Tuning tips:**
- **Robot can't turn well?** `wz_std` is the #1 knob. Increase it (0.4-1.0).
  - Low wz_std = most samples are near-straight trajectories.
  - High wz_std = more samples include sharp turns.
- **Robot wobbles/oscillates?** `wz_std` may be too high. Reduce to 0.3-0.5.
- **Robot doesn't vary speed?** Increase `vx_std` to 0.2-0.3.

---

## 4. Acceleration Limits

| Parameter | Current | Description |
|-----------|---------|-------------|
| `ax_max` | 2.5 | Max forward acceleration (m/s^2) |
| `ax_min` | -2.5 | Max braking deceleration |
| `az_max` | 3.2 | Max angular acceleration (rad/s^2) |

**Tuning tips:**
- Robot accelerates too aggressively? Reduce `ax_max` to 1.0-1.5.
- Robot can't stop in time? Increase `|ax_min|`.
- Turns feel sluggish to initiate? Increase `az_max`.

---

## 5. Critics (Cost Functions)

Critics evaluate each sampled trajectory. Total cost = sum of all enabled critics.
**Higher weight = stronger influence on trajectory selection.**

### 5.1 ConstraintCritic
Penalizes trajectories violating kinematic constraints.

| Parameter | Current | Notes |
|-----------|---------|-------|
| `cost_weight` | 4.0 | Usually leave at default |

### 5.2 GoalCritic
Pulls robot toward the goal position.

| Parameter | Current | Notes |
|-----------|---------|-------|
| `cost_weight` | 5.0 | Increase to make robot more goal-driven |
| `threshold_to_consider` | 1.4 | Only active within this distance to goal (m) |

### 5.3 GoalAngleCritic
Aligns robot heading with goal orientation at the end.

| Parameter | Current | Notes |
|-----------|---------|-------|
| `cost_weight` | 3.0 | Increase if final heading matters |
| `threshold_to_consider` | 0.5 | Only active within this distance to goal (m) |

### 5.4 PreferForwardCritic
Penalizes backward and sideways motion.

| Parameter | Current | Notes |
|-----------|---------|-------|
| `enabled` | true | **Set false for omnidirectional / equal forward-backward** |
| `cost_weight` | 5.0 | Lower = less penalty for reversing |
| `threshold_to_consider` | 0.5 | Only active within this distance to goal |

**Tuning tips:**
- Robot should back up freely? Set `enabled: false`.
- Robot reverses too eagerly? Increase weight or enable it.
- For skid-steer robots (like GO2W) that reverse well, disable or keep weight low (2-5).

### 5.5 CostCritic
Penalizes trajectories passing through high-cost (near-obstacle) costmap cells.
**This is the main obstacle avoidance critic.**

| Parameter | Current | Range | Notes |
|-----------|---------|-------|-------|
| `cost_weight` | 5.0 | 2-15 | Higher = stays further from obstacles |
| `consider_footprint` | false | bool | **true = checks full robot shape, not just center point** |
| `critical_cost` | 300.0 | 200-300 | Costmap value considered lethal |
| `near_collision_cost` | 240 | 150-253 | Start heavy penalty above this cost |
| `collision_cost` | 1e6 | large | Penalty multiplier for collision trajectories |
| `near_goal_distance` | 1.0 | | Relaxes cost checking near goal |

**Tuning tips:**
- **Robot clips obstacles?** Set `consider_footprint: true` (uses real shape instead of point).
- **Robot clips obstacles even with footprint?** Increase `cost_weight` to 8-12.
- **Robot won't go through narrow passages?** Reduce `cost_weight` or `near_collision_cost`.
- **"Start occupied" errors?** Robot center is on lethal cell. Fix laser filter / reduce robot_radius in costmap.

### 5.6 PathAlignCritic
Keeps robot aligned with the global path direction.

| Parameter | Current | Range | Notes |
|-----------|---------|-------|-------|
| `cost_weight` | 22.0 | 5-30 | Higher = sticks closer to path line |
| `max_path_occupancy_ratio` | 0.40 | 0.0-1.0 | Disables critic if this fraction of path is in collision. Low (0.05) = disables near obstacles. |
| `offset_from_furthest` | 20 | 5-30 | How many path points ahead to consider |

**Tuning tips:**
- Robot deviates from path too much? Increase weight.
- Robot gets stuck trying to align near obstacles? Reduce `max_path_occupancy_ratio` to 0.05.
- Weight too high + narrow corridor = robot won't deviate to avoid obstacles.

### 5.7 PathFollowCritic
Pulls robot toward the furthest reachable point on the path (carrot).

| Parameter | Current | Range | Notes |
|-----------|---------|-------|-------|
| `cost_weight` | 12.0 | 3-15 | Higher = chases the carrot more aggressively |
| `offset_from_furthest` | 20 | 5-30 | Steps back from furthest visible path point |
| `threshold_to_consider` | 1.4 | | Disabled within this distance to goal |

**Tuning tips:**
- Robot overshoots turns? Reduce weight or `offset_from_furthest`.
- Robot takes shortcuts cutting corners? Increase weight.

### 5.8 PathAngleCritic
Penalizes robot heading that deviates from the path direction.

| Parameter | Current | Range | Notes |
|-----------|---------|-------|-------|
| `cost_weight` | 2.0 | 1-25 | **High values resist turning.** |
| `max_angle_to_furthest` | 1.0 | rad | Max angle before max penalty |
| `mode` | 0 | 0/1/2 | 0=forward-only, 1=reverse-only, 2=bidirectional |
| `offset_from_furthest` | 4 | | Path points ahead to compute angle to |

**Tuning tips:**
- **Robot can't turn / is sluggish turning?** This is often the culprit. Reduce to 1-5.
- Robot zigzags? Increase weight to stabilize heading.
- For skid-steer: keep low (1-3), these robots need to turn sharply.

---

## 6. Other Parameters

| Parameter | Current | Notes |
|-----------|---------|-------|
| `prune_distance` | 2.5 | How far behind to trim the path (m) |
| `transform_tolerance` | 0.5 | Max age of TF transforms (s) |
| `gamma` | 0.015 | Controls cost normalization. Rarely needs changing. |
| `motion_model` | DiffDrive | Options: DiffDrive, Omni, Ackermann |

---

## 7. Common Scenarios & Fixes

### Robot collides with obstacles
1. `CostCritic.consider_footprint: true`
2. Increase `CostCritic.cost_weight` (8-12)
3. Increase inflation_radius in costmap (0.4-0.7)
4. Reduce `vx_max` (0.3-0.5 for offices)

### Robot can't turn / gets stuck at corners
1. Increase `wz_std` (0.4-1.0) -- most important
2. Reduce `PathAngleCritic.cost_weight` (1-3)
3. Reduce `PreferForwardCritic.cost_weight` or disable
4. Increase `batch_size` (2000-3000)
5. Increase `wz_max` if motor can handle it

### Robot oscillates / wobbles
1. Reduce `wz_std` (0.2-0.4)
2. Increase `PathAlignCritic.cost_weight`
3. Increase `PathAngleCritic.cost_weight` slightly
4. Reduce `vx_std`

### Robot won't reverse
1. Set `vx_min` to negative of `vx_max`
2. Disable `PreferForwardCritic` (`enabled: false`)
3. Set `PathAngleCritic.mode: 2` (bidirectional)

### Robot too slow
1. Increase `vx_max`
2. Reduce `CostCritic.cost_weight` (robot will cut closer to walls)
3. Reduce batch_size/iteration_count for lower compute

### Robot overshoots goals
1. Reduce `vx_max` or `ax_max`
2. Increase `GoalCritic.cost_weight`
3. Reduce `PathFollowCritic.offset_from_furthest`

### "Start occupied" planner error
1. Check laser_filter is removing self-occlusion hits
2. Reduce `robot_radius` in global_costmap (but not below true radius)
3. Increase `obstacle_min_range` in costmap
4. Verify costmap inflation_radius isn't causing robot center to be lethal

---

## 8. Critic Weight Balancing Guide

Think of critics as competing forces. The relative weights matter more than absolute values.

```
Obstacle avoidance vs Path following:
  CostCritic (5.0) vs PathAlignCritic (22.0) + PathFollowCritic (12.0)
  If CostCritic << PathFollow, robot will hug path even into obstacles.
  If CostCritic >> PathFollow, robot will deviate widely from path.

Turning ability vs Heading stability:
  wz_std (noise) vs PathAngleCritic (weight)
  High wz_std + low PathAngle = agile but potentially wobbly
  Low wz_std + high PathAngle = stable but sluggish turning

Forward preference vs Maneuverability:
  PreferForwardCritic vs vx_min (reverse limit)
  Disable PreferForward + large |vx_min| = full omnidirectional
  High PreferForward + small |vx_min| = forward-only robot
```

---

## 9. Current GO2W Config Summary

```
Horizon:     56 steps x 0.05s = 2.8s lookahead
Samples:     2000 trajectories, 2 iterations
Speed:       1.0 m/s forward, 1.0 m/s reverse
Turn rate:   1.9 rad/s max, wz_std=0.8
Key critics: PathAlign(22) > PathFollow(12) > CostCritic(5) > PreferFwd(5)
```
