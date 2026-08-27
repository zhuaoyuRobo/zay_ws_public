# Go2-Class ROS 2 Navigation Workspace

This repository is a public, simulation-first ROS 2 workspace for integrating
a Go2-class mobile robot with FAST-LIO, point-cloud localization, Nav2, and an
optional learned locomotion controller.

The public tree contains algorithm code and generic interfaces only. It does
not contain laboratory identities, host addresses, device serials, real-world
maps, recorded data, policy weights, vendor SDK binaries, or calibrated robot
parameters.

## Repository layout

- `src/isaac_go2_nav2`: integration nodes and simulation launch files.
- `src/go2_description`: robot description resources.
- `src/FAST_LIO`: upstream FAST-LIO source.
- `src/livox_ros_driver2`: upstream Livox ROS 2 driver.
- `src/lidar_localization_ros2`: point-cloud localization components.
- `src/ndt_omp_ros2`: NDT registration components.
- `scripts`: repository-level build and public-audit helpers.

Third-party packages retain their own copyright notices and licenses. See
`THIRD_PARTY.md` before redistributing the repository.

## Public/private boundary

The following artifacts are intentionally excluded:

- real-robot launch files and calibrated Go2W/MID360 parameter sets;
- maps, point clouds, pose graphs, bags, and experiment logs;
- policy checkpoints and policy-environment configuration;
- vendor SDKs, binary dependencies, build products, and virtual environments;
- private network addresses, usernames, absolute workstation paths, and lab
  documentation.

Put local values under `config/private/`. That directory is ignored by Git,
except for its explanatory README. Do not move tuned parameters into tracked
launch defaults.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`, `rosdep`, and the dependencies declared by each ROS package
- an external simulator or sensor source publishing the topics required by the
  selected launch file

## Build

```bash
bash scripts/build_workspace.sh
source install/setup.bash
```

The helper runs `rosdep` and a symlinked `colcon` build. You can also build the
workspace manually:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Simulation use

The repository does not bundle a map or locomotion policy. Supply your own
compatible files explicitly:

```bash
ros2 launch isaac_go2_nav2 isaac_nav2.launch.py \
  map:=/absolute/path/to/map.yaml \
  policy_path:=/absolute/path/to/policy.pt \
  env_config_path:=/absolute/path/to/policy_env.yaml \
  use_sim_time:=true
```

The public policy action scale defaults to zero. Hardware command output is
also disabled by default, and the included command limits are deliberately
slow demonstration values. They are not a substitute for platform-specific
calibration, independent emergency-stop hardware, or controlled validation.

## Before publishing a fork

Run the local audit and inspect the result:

```bash
bash scripts/audit_public_tree.sh
```

Also review the Git diff for maps, weights, log archives, host addresses,
personal paths, and device identifiers before every push.

## License

The integration package `isaac_go2_nav2` declares Apache-2.0. Vendored and
upstream packages are governed by their own license files. No license claim in
this README overrides those files.
