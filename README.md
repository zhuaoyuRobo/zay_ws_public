# Go2W ROS 2 Navigation Workspace

A ROS 2 workspace for Go2W-class robot navigation, integrating FAST-LIO,
point-cloud localization, Nav2, target following, and an optional learned
locomotion controller.

This public repository is derived from real-world integration and testing. It
contains the algorithm code, ROS interfaces, launch files, and a sanitized set
of executable YAML parameters needed to study and reproduce the software
pipeline. Hardware command output is disabled by default.

## Features

- FAST-LIO odometry and LiDAR-inertial state estimation
- point-cloud registration and localization components
- Nav2 mapping, planning, and navigation integration
- point-cloud-to-laser-scan conversion with robot-body filtering
- odometry validation, filtering, and TF publication
- target-following and velocity-command processing nodes
- optional learned locomotion-policy interface
- simulation and public-data launch paths for software verification
- conservative public runtime parameters and repository audit tools

## Repository layout

- `src/isaac_go2_nav2`: integration nodes, launch files, parameters, and RViz
  resources.
- `src/go2_description`: Go2-class robot descriptions and simulation assets.
- `src/FAST_LIO`: upstream FAST-LIO source.
- `src/livox_ros_driver2`: upstream Livox ROS 2 driver.
- `src/lidar_localization_ros2`: point-cloud localization components.
- `src/ndt_omp_ros2`: multithreaded NDT/GICP registration components.
- `scripts`: workspace build and public-tree audit helpers.
- `config/private`: ignored location for local, non-public configuration.

Third-party packages retain their own copyright notices and licenses. Review
`THIRD_PARTY.md` before redistributing the workspace.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- `colcon` and `rosdep`
- dependencies declared by the individual ROS packages
- a compatible simulator, recorded dataset, or sensor source

## Clone and build

```bash
git clone https://github.com/zhuaoyuRobo/Go2W-navigate-project.git
cd Go2W-navigate-project

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The repository also provides a helper script:

```bash
bash scripts/build_workspace.sh
source install/setup.bash
```

## Public reproduction parameters

The tracked profile at
`src/isaac_go2_nav2/config/public_reproduction.yaml` contains sanitized,
executable parameters derived from integration testing. It covers:

- velocity limiting and command timeout behavior;
- odometry filtering and rejection thresholds;
- TF and simulated odometry publication;
- point-cloud projection and self-filtering;
- target-following update rules; and
- conservative Go2W command-bridge limits.

For example:

```bash
PARAMS="$(ros2 pkg prefix isaac_go2_nav2)/share/isaac_go2_nav2/config/public_reproduction.yaml"

ros2 run isaac_go2_nav2 cmd_vel_limiter \
  --ros-args --params-file "${PARAMS}"
```

The same file can be passed to other named nodes in the profile, including
`fastlio_nav2_odom_filter`, `stage2_pointcloud_to_laserscan`,
`person_target_follower`, and `go2w_cmd_vel_bridge`.

These values are public reproduction settings, not a complete real-robot
calibration. The profile uses simulated time, keeps the Go2W command bridge
disabled, and applies deliberately conservative command limits. Review and
validate every parameter before using it on hardware.

## Example launch paths

### Nav2 with a user-supplied map and optional policy

```bash
ros2 launch isaac_go2_nav2 isaac_nav2.launch.py \
  map:=/absolute/path/to/map.yaml \
  policy_path:=/absolute/path/to/policy.pt \
  env_config_path:=/absolute/path/to/policy_env.yaml \
  use_sim_time:=true
```

The policy is optional. Public defaults set the learned-policy action scale to
zero, so providing a checkpoint alone does not enable motion output.

### FAST-LIO and Nav2

```bash
ros2 launch isaac_go2_nav2 fastlio_os0_nav2.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_sim_time:=true
```

Sensor topics, frames, initial pose, and Nav2 parameters must match the selected
simulator, dataset, or robot configuration.

## Real-robot use

Real-robot deployment requires a separate local configuration containing the
correct sensor topics, frame transforms, map, initial pose, command interface,
device-specific calibration, and tested safety limits. Store those values in
`config/private/` or another ignored path.

Before enabling hardware commands:

1. Verify localization and TF without motion output.
2. Validate command signs, frame conventions, and timeout behavior.
3. Start with low speed and acceleration limits in a controlled area.
4. Use an independent emergency stop and a human safety operator.
5. Enable the command bridge only after the complete chain has been checked.

## Public/private boundary

The public repository intentionally excludes:

- laboratory identities and internal documentation;
- host addresses, credentials, device serial numbers, and private network data;
- real-world maps, point clouds, pose graphs, rosbags, and experiment logs;
- policy checkpoints and private policy-environment configurations;
- calibrated Go2W/MID360 deployment profiles;
- vendor SDK binaries, build products, and virtual environments; and
- absolute workstation paths or user-specific launch files.

The tracked YAML parameters do not include a sensor address, map, policy
checkpoint, credential, or complete hardware calibration.

## Safety notice

This repository is intended for research, education, and controlled testing.
The supplied parameters are not a safety certification and are not guaranteed
to suit another robot, sensor mounting arrangement, or environment. Users are
responsible for hardware validation, collision prevention, emergency-stop
provisions, and compliance with applicable rules.

## Before publishing a fork

Run the local audit and inspect the Git diff:

```bash
bash scripts/audit_public_tree.sh
git status
git diff --cached
```

Check for maps, model weights, archives, logs, addresses, personal paths, and
device identifiers before every push.

## License

The repository-level license is Apache-2.0. Vendored and upstream packages are
governed by their own license files; no statement in this README overrides
those terms.
