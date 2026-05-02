# FusionCore + icp_odometry: TurtleBot3 Gazebo Demo

This demo shows a feedback loop between [FusionCore](https://github.com/manankharwar/fusioncore) and rtabmap's `icp_odometry` where each node tightens the other.

## Architecture

```
/imu  ──────────────────────┐
/odom (wheel) ──────────────┤──→  FusionCore (UKF)
/rtabmap/icp_odometry ──────┘     │
    ↑                             │ publishes: odom → base_footprint TF
    │                             │            /fusion/odom
    │         guess_frame_id: odom│
    └──────  icp_odometry ←───────┘
                │  (publish_tf: false)
                │
                └──→ /rtabmap/icp_odometry ──→ rtabmap SLAM ──→ map → odom TF
```

**What each node contributes:**

| Node | Input | Provides |
|---|---|---|
| FusionCore | wheels + IMU | stable `odom` frame, continuous state at 100 Hz |
| icp_odometry | `/scan` + FusionCore's `odom` as initial guess | scan-level pose corrections |
| FusionCore encoder2 | icp_odometry output | tighter velocity corrections from ICP |
| rtabmap SLAM | icp_odometry output | global map, loop closures |

FusionCore gives `icp_odometry` a stable initial guess via `guess_frame_id: odom`.
Better initial guesses mean scan matching succeeds more often and with lower error.
The ICP result feeds back into FusionCore as a second velocity source (`encoder2`),
tightening the state estimate further. `Odom/ResetCountdown: 1` lets the system
auto-recover if ICP loses tracking.

## Quick start

```bash
export TURTLEBOT3_MODEL=waffle

ros2 launch rtabmap_demos turtlebot3_sim_fusioncore_icp_demo.launch.py
```

Optional arguments:

```bash
# Localization mode (requires saved map from a previous mapping run)
ros2 launch rtabmap_demos turtlebot3_sim_fusioncore_icp_demo.launch.py localization:=true

# Different Gazebo world
ros2 launch rtabmap_demos turtlebot3_sim_fusioncore_icp_demo.launch.py world:=house
```

## Prerequisites

```bash
sudo apt install ros-jazzy-fusioncore-ros ros-jazzy-turtlebot3-gazebo \
                 ros-jazzy-rtabmap-ros ros-jazzy-nav2-bringup
export TURTLEBOT3_MODEL=waffle
```

## Files

| File | Purpose |
|---|---|
| `turtlebot3_sim_fusioncore_icp_demo.launch.py` | Complete demo: Gazebo + FusionCore + rtabmap + Nav2 |
| `turtlebot3_fusioncore_icp.launch.py` | Core only: FusionCore + icp_odometry + rtabmap (no Gazebo) |
| `../../params/fusioncore_tb3.yaml` | FusionCore config for TB3 Waffle |
| `../../params/turtlebot3_fusioncore_icp_nav2_params.yaml` | Nav2 config using `/fusion/odom` |

## Topic and TF summary

| Topic / TF | Publisher | Subscribers |
|---|---|---|
| `/imu` | Gazebo | FusionCore |
| `/odom` | Gazebo (wheel) | FusionCore |
| `/scan` | Gazebo (lidar) | icp_odometry, rtabmap |
| `/rtabmap/icp_odometry` | icp_odometry | FusionCore (encoder2), rtabmap |
| `/fusion/odom` | FusionCore | Nav2 |
| TF `odom → base_footprint` | FusionCore | icp_odometry (guess), Nav2 |
| TF `map → odom` | rtabmap | Nav2 |
