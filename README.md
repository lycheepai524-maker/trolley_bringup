# trolley_bringup

Full-system bringup package for the Trolley robot: rosserial, IMU, EKF, encoder odometry, YDLidar, SLAM, navigation, and WiFi-based localization.

## Dependencies

- ROS Noetic
- `rosserial_python`, `imu_filter_madgwick`, `robot_localization`
- `ydlidar_ros_driver`, `slam_toolbox`, `trolley_slam`

## Launch files

| File | Purpose |
|---|---|
| `trolley_bringup.launch` | Core bringup: rosserial + IMU + EKF + encoder odom + LiDAR |
| `trolley_navigation.launch` | Navigation stack (move_base) with a saved map |
| `trolley_explore.launch` | Autonomous exploration with slam_toolbox |
| `trolley_random_explore.launch` | Random frontier exploration |
| `trolley_follower.launch` | Person/object follower |
| `trolley_obstacle_avoidance.launch` | Reactive obstacle avoidance |
| `trolley_bringup_hector.launch` | Bringup variant using Hector SLAM |

## Scripts

| Script | Purpose |
|---|---|
| `wifi_initial_pose.py` | Scan WiFi, match against fingerprint DB, publish `/initialpose` |
| `wifi_locate.py` | One-shot WiFi localisation |
| `wifi_resolver.py` | WiFi fingerprint collection helper |
| `encoder_to_odom.py` | Convert encoder ticks to `nav_msgs/Odometry` |
| `record_location.py` | Save the current pose as a named waypoint |
| `goto_location.py` | Send the robot to a saved waypoint |
| `lidar_follower.py` | Follow the nearest LiDAR cluster |
| `trolley_command_node.py` | High-level command interface |
| `cmd_vel_boost.py` | Scale `cmd_vel` for speed tuning |
| `calibrate_ticks.py` | Measure encoder ticks-per-metre |
| `drive_test.py` | Basic drive test routine |
| `random_explorer.py` | Random-walk exploration node |

## WiFi fingerprint database

Fingerprints are stored in `~/trolley_fingerprints.db` (SQLite).

| Table | Columns |
|---|---|
| `phone_points` | `point_id`, `x`, `y`, `label`, `created_at` |
| `phone_reading` | `point_id`, `bssid`, `rssi`, `ssid` |

Collect fingerprints with `wifi_resolver.py`, then run `wifi_initial_pose.py` at startup to seed AMCL with a WiFi-derived initial pose.

## Quick start

```bash
roslaunch trolley_bringup trolley_bringup.launch
roslaunch trolley_bringup trolley_navigation.launch
rosrun trolley_bringup wifi_initial_pose.py
```
