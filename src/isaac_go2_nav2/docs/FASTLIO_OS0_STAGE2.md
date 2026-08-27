# FAST_LIO OS0 第二阶段路线

第二阶段与第一阶段的 MID360 路线相互隔离。它使用 Isaac 中类似 OS0 的
RTX 激光雷达和 IMU 发布器，然后由 FAST_LIO 为 Nav2 提供里程计和局部配准点云。

```text
Isaac OS0 RTX lidar
  -> /os_cloud_node/points
  -> FAST_LIO
  -> 原始 /Odometry 和 camera_init -> body TF
  -> fastlio_nav2_odom_filter
  -> /Odometry_nav2 和 camera_init -> body_nav TF
  -> stage2_pointcloud_to_laserscan 读取原始 /os_cloud_node/points
  -> TF 转换到 body_nav 后投影
  -> /scan（frame=body_nav）
  -> Nav2 navigation stack
  -> /cmd_vel
  -> Isaac 官方 Go2 policy
```

第二阶段相关文件：

```text
bringup/start_isaac_go2_os0_gui.sh
bringup/start_fastlio_os0_nav2.sh
ros2_ws/scripts/run_fastlio_os0_nav2.sh
ros2_ws/src/isaac_go2_nav2/launch/fastlio_os0_nav2.launch.py
ros2_ws/src/isaac_go2_nav2/config/fast_lio_os0_isaac.yaml
ros2_ws/src/isaac_go2_nav2/config/nav2_fastlio_os0_params.yaml
ros2_ws/src/isaac_go2_nav2/isaac_go2_nav2/fastlio_nav2_odom_filter.py
ros2_ws/src/isaac_go2_nav2/isaac_go2_nav2/stage2_pointcloud_to_laserscan.py
```

启动 Isaac：

```bash
bash bringup/start_isaac_go2_os0_gui.sh
```

启动 FAST_LIO + Nav2：

```bash
bash bringup/start_fastlio_os0_nav2.sh rviz:=true
```

预期 topic：

```text
/clock
/os_cloud_node/points
/os_cloud_node/imu
/Odometry
/Odometry_nav2
/cloud_registered_body
/scan
/tf
/cmd_vel
```

坐标系：

```text
map -> camera_init
camera_init -> body           # FAST_LIO 原始 6DoF
camera_init -> body_nav       # Nav2 使用的 2D/边界过滤后位姿
body -> os_sensor
```

静态 `map -> camera_init` 变换用于应用场景中的初始生成偏移，
`camera_init -> body` 由 FAST_LIO 负责发布；Nav2 使用过滤后的
`camera_init -> body_nav` 和 `/Odometry_nav2`。这条路线不要启动 AMCL。

当前 `/scan` 默认从原始 OS0 点云生成，而不是从 `/cloud_registered_body`
生成。原因是 FAST_LIO 的 `blind` 参数会丢掉近距离点；局部避障需要更近的
障碍信息。生成 `/scan` 前会先把点云 TF 到 `body_nav`，再过滤地面低点和
机器人自身 footprint 内的点，避免把地面或自车点误标成障碍。FAST_LIO
仍然使用自己的 blind 设置保证建图稳定。

仿真中 `fastlio_nav2_odom_filter` 会同时订阅 Isaac 真值 `/odom` 做诊断：
当 FAST_LIO/Nav2 位姿和真值偏差超过阈值时，日志会打印距离和 yaw 偏差。
