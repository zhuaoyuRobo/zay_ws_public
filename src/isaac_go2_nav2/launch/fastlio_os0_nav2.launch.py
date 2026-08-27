from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("isaac_go2_nav2"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    fast_lio_share = Path(get_package_share_directory("fast_lio"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    fast_lio_rviz = LaunchConfiguration("fast_lio_rviz")
    autostart = LaunchConfiguration("autostart")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    fast_lio_config_path = LaunchConfiguration("fast_lio_config_path")
    fast_lio_config_file = LaunchConfiguration("fast_lio_config_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    scan_use_tf = LaunchConfiguration("scan_use_tf")
    base_frame = LaunchConfiguration("base_frame")
    nav_base_frame = LaunchConfiguration("nav_base_frame")
    nav_odom_topic = LaunchConfiguration("nav_odom_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    scan_stamp_mode = LaunchConfiguration("scan_stamp_mode")
    log_level = LaunchConfiguration("log_level")
    use_respawn = LaunchConfiguration("use_respawn")
    reject_out_of_map = LaunchConfiguration("reject_out_of_map")
    external_pose_correction = LaunchConfiguration("external_pose_correction")
    external_pose_topic = LaunchConfiguration("external_pose_topic")
    external_pose_type = LaunchConfiguration("external_pose_type")
    external_pose_mode = LaunchConfiguration("external_pose_mode")
    external_correction_alpha = LaunchConfiguration("external_correction_alpha")
    external_correction_rate = LaunchConfiguration("external_correction_rate")
    external_max_correction_step = LaunchConfiguration("external_max_correction_step")
    external_max_yaw_correction_step = LaunchConfiguration("external_max_yaw_correction_step")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("fast_lio_rviz", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Path to a user-supplied Nav2 map YAML.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=str(pkg_share / "config" / "nav2_fastlio_os0_params.yaml"),
            ),
            DeclareLaunchArgument(
                "fast_lio_config_path",
                default_value=str(pkg_share / "config"),
            ),
            DeclareLaunchArgument("fast_lio_config_file", default_value="fast_lio_os0_isaac.yaml"),
            DeclareLaunchArgument("cloud_topic", default_value="/os_cloud_node/points"),
            DeclareLaunchArgument("imu_topic", default_value="/os_cloud_node/imu"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/os_cloud_node/points"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_use_tf", default_value="true"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            DeclareLaunchArgument("nav_base_frame", default_value="body_nav"),
            DeclareLaunchArgument("nav_odom_topic", default_value="/Odometry_nav2"),
            DeclareLaunchArgument("truth_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("lidar_frame", default_value="os_sensor"),
            DeclareLaunchArgument("initial_x", default_value="-6.0"),
            DeclareLaunchArgument("initial_y", default_value="-4.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("truth_initial_x", default_value="-6.0"),
            DeclareLaunchArgument("truth_initial_y", default_value="-4.0"),
            DeclareLaunchArgument("truth_initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("lidar_x", default_value="0.24"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.25"),
            DeclareLaunchArgument("min_height", default_value="0.05"),
            DeclareLaunchArgument("max_height", default_value="0.85"),
            DeclareLaunchArgument("angle_min", default_value="-3.14159265359"),
            DeclareLaunchArgument("angle_max", default_value="3.14159265359"),
            DeclareLaunchArgument("angle_increment", default_value="0.00872664626"),
            DeclareLaunchArgument("scan_time", default_value="0.10"),
            DeclareLaunchArgument("scan_stamp_mode", default_value="cloud"),
            DeclareLaunchArgument("range_min", default_value="0.30"),
            DeclareLaunchArgument("range_max", default_value="12.0"),
            DeclareLaunchArgument("reject_out_of_map", default_value="true"),
            DeclareLaunchArgument("external_pose_correction", default_value="false"),
            DeclareLaunchArgument("external_pose_topic", default_value="/odom"),
            DeclareLaunchArgument("external_pose_type", default_value="odometry"),
            DeclareLaunchArgument("external_pose_mode", default_value="relative"),
            DeclareLaunchArgument("external_correction_alpha", default_value="0.2"),
            DeclareLaunchArgument("external_correction_rate", default_value="20.0"),
            DeclareLaunchArgument("external_max_correction_step", default_value="0.5"),
            DeclareLaunchArgument("external_max_yaw_correction_step", default_value="0.6"),
            Node(
                condition=UnlessCondition(external_pose_correction),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_fastlio_init_tf",
                arguments=[
                    "--x",
                    LaunchConfiguration("initial_x"),
                    "--y",
                    LaunchConfiguration("initial_y"),
                    "--z",
                    LaunchConfiguration("initial_z"),
                    "--yaw",
                    LaunchConfiguration("initial_yaw"),
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "camera_init",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_os0_tf",
                arguments=[
                    "--x",
                    LaunchConfiguration("lidar_x"),
                    "--y",
                    LaunchConfiguration("lidar_y"),
                    "--z",
                    LaunchConfiguration("lidar_z"),
                    "--yaw",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    base_frame,
                    "--child-frame-id",
                    lidar_frame,
                ],
                output="screen",
            ),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                output="screen",
                parameters=[
                    PathJoinSubstitution([fast_lio_config_path, fast_lio_config_file]),
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "common.lid_topic": cloud_topic,
                        "common.imu_topic": imu_topic,
                    },
                ],
            ),
            Node(
                package="isaac_go2_nav2",
                executable="fastlio_nav2_odom_filter",
                name="fastlio_nav2_odom_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_odom_topic": "/Odometry",
                        "output_odom_topic": nav_odom_topic,
                        "odom_frame": "camera_init",
                        "nav_base_frame": nav_base_frame,
                        "map_yaml": map_file,
                        "initial_x": ParameterValue(LaunchConfiguration("initial_x"), value_type=float),
                        "initial_y": ParameterValue(LaunchConfiguration("initial_y"), value_type=float),
                        "initial_yaw": ParameterValue(LaunchConfiguration("initial_yaw"), value_type=float),
                        "truth_initial_x": ParameterValue(
                            LaunchConfiguration("truth_initial_x"),
                            value_type=float,
                        ),
                        "truth_initial_y": ParameterValue(
                            LaunchConfiguration("truth_initial_y"),
                            value_type=float,
                        ),
                        "truth_initial_yaw": ParameterValue(
                            LaunchConfiguration("truth_initial_yaw"),
                            value_type=float,
                        ),
                        "truth_odom_topic": LaunchConfiguration("truth_odom_topic"),
                        "reject_out_of_map": ParameterValue(reject_out_of_map, value_type=bool),
                    }
                ],
            ),
            Node(
                condition=IfCondition(external_pose_correction),
                package="isaac_go2_nav2",
                executable="external_pose_map_corrector",
                name="external_pose_map_corrector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "local_odom_topic": nav_odom_topic,
                        "external_pose_topic": external_pose_topic,
                        "external_pose_type": external_pose_type,
                        "external_pose_mode": external_pose_mode,
                        "map_frame": "map",
                        "odom_frame": "camera_init",
                        "base_frame": nav_base_frame,
                        "external_origin_x": ParameterValue(
                            LaunchConfiguration("truth_initial_x"),
                            value_type=float,
                        ),
                        "external_origin_y": ParameterValue(
                            LaunchConfiguration("truth_initial_y"),
                            value_type=float,
                        ),
                        "external_origin_yaw": ParameterValue(
                            LaunchConfiguration("truth_initial_yaw"),
                            value_type=float,
                        ),
                        "correction_alpha": ParameterValue(
                            external_correction_alpha,
                            value_type=float,
                        ),
                        "max_correction_step": ParameterValue(
                            external_max_correction_step,
                            value_type=float,
                        ),
                        "max_yaw_correction_step": ParameterValue(
                            external_max_yaw_correction_step,
                            value_type=float,
                        ),
                        "publish_rate": ParameterValue(
                            external_correction_rate,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="isaac_go2_nav2",
                executable="stage2_pointcloud_to_laserscan",
                name="stage2_os0_pointcloud_to_laserscan",
                output="screen",
                remappings=[
                    ("cloud_in", scan_cloud_topic),
                    ("scan", scan_topic),
                ],
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "target_frame": nav_base_frame,
                        "transform_tolerance": 0.10,
                        "use_tf": ParameterValue(scan_use_tf, value_type=bool),
                        "stamp_mode": scan_stamp_mode,
                        "min_height": ParameterValue(LaunchConfiguration("min_height"), value_type=float),
                        "max_height": ParameterValue(LaunchConfiguration("max_height"), value_type=float),
                        "angle_min": ParameterValue(LaunchConfiguration("angle_min"), value_type=float),
                        "angle_max": ParameterValue(LaunchConfiguration("angle_max"), value_type=float),
                        "angle_increment": ParameterValue(
                            LaunchConfiguration("angle_increment"),
                            value_type=float,
                        ),
                        "scan_time": ParameterValue(LaunchConfiguration("scan_time"), value_type=float),
                        "range_min": ParameterValue(LaunchConfiguration("range_min"), value_type=float),
                        "range_max": ParameterValue(LaunchConfiguration("range_max"), value_type=float),
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
            ),
            Node(
                condition=IfCondition(fast_lio_rviz),
                package="rviz2",
                executable="rviz2",
                name="fast_lio_rviz2",
                arguments=["-d", str(fast_lio_share / "rviz" / "fastlio.rviz")],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "yaml_filename": map_file,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": ParameterValue(autostart, value_type=bool),
                        "node_names": ["map_server"],
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params_file,
                    "autostart": autostart,
                    "use_composition": "False",
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                }.items(),
            ),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(nav2_share / "rviz" / "nav2_default_view.rviz")],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                output="screen",
            ),
        ]
    )
