# multi_robot_slam_toolbox_simulation

**multi-robot SLAM demo** for ROS 2 using `slam_toolbox`.
Based on [acachathuranga/slam_toolbox_multi_robot_demo](https://github.com/acachathuranga/slam_toolbox_multi_robot_demo), adapted here for a shared single-Gazebo-instance setup on ROS 2 Jazzy.

Launch 2 TurtleBot3 robots in a single shared Gazebo (gz sim) instance, each with a namespaced `slam_toolbox` instance and a shared **global odometry** frame (`global_odom → map`).

Map merging across robots is **not** handled in this version of the package — see `map_merge_server` for that.

---

![multirobot_slam_simulation](images/simulation.gif?raw=true "slam_toolbox_multi_robot_demo")

## Features

- Single-robot or **multi-robot** bringup (2 robots) with per-robot namespaces.
- Single shared Gazebo world and single shared simulation clock for all robots (a per-robot Gazebo/clock setup caused clock desync and TF conflicts between robots).
- Starts `slam_toolbox` (standard, per-robot instance) per robot. The decentralized multi-robot variant isn't shipped with this ROS distro, so this demo copies and adapts the closest available launch to reach an equivalent per-robot setup.
- Publishes **static TF** per robot: `global_odom → map` (not `→ odom`), so each robot's own live `map → odom` correction from `slam_toolbox` stays the only source of truth for that link.
- Optional RViz per robot with a default config.

---

## Dependencies

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-diff-drive-controller ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-teleop-twist-keyboard
```

---

## Build & Source

```bash
# From your workspace root
colcon build --packages-select slam_toolbox_multi_robot_demo
source install/setup.bash
```
