# ndt_omp_ros2

[![CI](https://github.com/rsasaki0109/ndt_omp_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/ndt_omp_ros2/actions/workflows/ci.yml)

This package provides an OpenMP-boosted Normal Distributions Transform (and GICP) algorithm derived from pcl. The NDT algorithm is modified to be SSE-friendly and multi-threaded. It can run up to 10 times faster than its original version in pcl.

## Supported ROS distributions

- ROS 2 Humble
- ROS 2 Jazzy

## Build from source

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ndt_omp_ros2
source install/setup.bash
```

The package installs and exports the `ndt_omp_ros2::ndt_omp` CMake target.
A downstream package can use it with:

```cmake
find_package(ndt_omp_ros2 REQUIRED)
target_link_libraries(my_registration_node ndt_omp_ros2::ndt_omp)
```

## Release preflight

The release gate generates Debian metadata with Bloom from the current clean
Git commit, builds the binary package, and verifies its identity and installed
library, executable, and manifest. Run it in the matching official ROS image:

```bash
apt-get update
apt-get install -y python3-bloom python3-jsonschema fakeroot debhelper
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths . --ignore-src -r -y \
  --rosdistro "$ROS_DISTRO"
python3 scripts/check_bloom_release.py \
  --ros-distro "$ROS_DISTRO" \
  --os-version "$OS_VERSION" \
  --evidence-dir "/tmp/ndt-omp-bloom-$ROS_DISTRO"
python3 -m jsonschema \
  --instance "/tmp/ndt-omp-bloom-$ROS_DISTRO/bloom_release_report.json" \
  schemas/bloom-release-v1.schema.json
```

Use `ROS_DISTRO=humble OS_VERSION=jammy` or
`ROS_DISTRO=jazzy OS_VERSION=noble`. The evidence directory must be new or
empty. It contains the JSON report, command logs, hashes, and the runtime
`.deb`; CI uploads the directory for each supported distribution. This gate is
a release prerequisite, but it does not create a tag or publish to rosdistro.
Maintainer-run results and artifact hashes are recorded in
[`docs/release-evidence.md`](docs/release-evidence.md).

## Benchmark

The repository includes two sample point clouds for the benchmark executable:

```bash
cd ~/ros2_ws/src/ndt_omp_ros2/data
ros2 run ndt_omp_ros2 align 251370668.pcd 251371071.pcd
```

```text
--- pcl::GICP ---
single : 267.385[msec]
10times: 1151.76[msec]
fitness: 0.220382

--- pclomp::GICP ---
single : 173.152[msec]
10times: 1299.14[msec]
fitness: 0.220388

--- pcl::NDT ---
single : 425.142[msec]
10times: 3638.77[msec]
fitness: 0.213937

--- pclomp::NDT (KDTREE, 1 threads) ---
single : 308.935[msec]
10times: 3095.53[msec]
fitness: 0.213937

--- pclomp::NDT (DIRECT7, 1 threads) ---
single : 188.942[msec]
10times: 1373.47[msec]
fitness: 0.214205

--- pclomp::NDT (DIRECT1, 1 threads) ---
single : 41.3584[msec]
10times: 347.261[msec]
fitness: 0.208511

--- pclomp::NDT (KDTREE, 8 threads) ---
single : 108.68[msec]
10times: 1046.16[msec]
fitness: 0.213937

--- pclomp::NDT (DIRECT7, 8 threads) ---
single : 56.9189[msec]
10times: 545.279[msec]
fitness: 0.214205

--- pclomp::NDT (DIRECT1, 8 threads) ---
single : 16.7266[msec]
10times: 169.097[msec]
fitness: 0.208511
```

Several methods for neighbor voxel search are implemented. If you select pclomp::KDTREE, results will be completely same as the original pcl::NDT. We recommend to use pclomp::DIRECT7 which is faster and stable. If you need extremely fast registration, choose pclomp::DIRECT1, but it might be a bit unstable.

<img src="data/screenshot.png" height="400pix" /><br>
Red: target, Green: source, Blue: aligned

## License

BSD-2-Clause. This project is a ROS 2 port of
[koide3/ndt_omp](https://github.com/koide3/ndt_omp).
