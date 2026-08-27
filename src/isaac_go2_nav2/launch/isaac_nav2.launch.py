from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


POLICY_CONTROLLER_BACKENDS = {
    "isaac_lab_go2",
    "isaac_lab",
    "isaac_official_go2",
    "internutopia_isaac_go2",
}

ALLOWED_LOCOMOTION_BACKENDS = POLICY_CONTROLLER_BACKENDS | {
    "isaac_internal",
}


def validate_locomotion_backend(context, *args, **kwargs):
    backend = LaunchConfiguration("locomotion_backend").perform(context)
    if backend in ("rl_sar", "legacy_rl_sar"):
        raise RuntimeError(
            "rl_sar/himloco.pt is the old Gazebo failed path and is blocked. "
            "Use locomotion_backend:=isaac_lab_go2."
        )
    if backend not in ALLOWED_LOCOMOTION_BACKENDS:
        raise RuntimeError(
            "Unsupported locomotion_backend %r. Allowed: %s"
            % (backend, ", ".join(sorted(ALLOWED_LOCOMOTION_BACKENDS)))
        )
    return []


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("isaac_go2_nav2"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    locomotion_backend = LaunchConfiguration("locomotion_backend")
    policy_path = LaunchConfiguration("policy_path")
    env_config_path = LaunchConfiguration("env_config_path")
    joint_command_topic = LaunchConfiguration("joint_command_topic")
    start_target_follower = LaunchConfiguration("target_follower")
    target_topic = LaunchConfiguration("target_topic")
    target_frame = LaunchConfiguration("target_frame")
    follow_distance = LaunchConfiguration("follow_distance")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    simulated_odom = LaunchConfiguration("simulated_odom")
    synthetic_scan = LaunchConfiguration("synthetic_scan")
    synthetic_imu = LaunchConfiguration("synthetic_imu")
    odom_topic = LaunchConfiguration("odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    static_laser_tf = LaunchConfiguration("static_laser_tf")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Path to a user-supplied Nav2 map YAML.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(pkg_share / "config" / "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "policy_path",
                default_value="",
                description="Path to a user-supplied locomotion policy.",
            ),
            DeclareLaunchArgument(
                "env_config_path",
                default_value="",
                description="Path to the matching user-supplied policy environment file.",
            ),
            DeclareLaunchArgument("locomotion_backend", default_value="isaac_lab_go2"),
            DeclareLaunchArgument("joint_command_topic", default_value="/joint_command"),
            DeclareLaunchArgument("policy_runtime", default_value="auto"),
            DeclareLaunchArgument("policy_strict_env_contract", default_value="true"),
            DeclareLaunchArgument("policy_action_scale", default_value="0.0"),
            DeclareLaunchArgument("policy_action_clip", default_value="0.0"),
            DeclareLaunchArgument("policy_safe_joint_clip", default_value="true"),
            DeclareLaunchArgument("policy_target_blend", default_value="1.0"),
            DeclareLaunchArgument("policy_target_step_limit", default_value="0.05"),
            DeclareLaunchArgument("policy_linear_command_gain", default_value="1.0"),
            DeclareLaunchArgument("policy_angular_command_gain", default_value="1.0"),
            DeclareLaunchArgument("policy_max_policy_linear", default_value="0.0"),
            DeclareLaunchArgument("policy_max_policy_angular", default_value="0.0"),
            DeclareLaunchArgument("policy_command_deadband", default_value="0.05"),
            DeclareLaunchArgument("policy_run_when_idle", default_value="false"),
            DeclareLaunchArgument("policy_joint_order", default_value="isaac_breadth"),
            DeclareLaunchArgument("joint_command_order", default_value="isaac_breadth"),
            DeclareLaunchArgument("target_follower", default_value="false"),
            DeclareLaunchArgument("target_topic", default_value="/person_pose"),
            DeclareLaunchArgument("target_frame", default_value=""),
            DeclareLaunchArgument("follow_distance", default_value="2.0"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("simulated_odom", default_value="false"),
            DeclareLaunchArgument("synthetic_scan", default_value="false"),
            DeclareLaunchArgument("synthetic_imu", default_value="false"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("flatten_odom_tf", default_value="true"),
            DeclareLaunchArgument("static_laser_tf", default_value="true"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("laser_frame", default_value="base_laser"),
            OpaqueFunction(function=validate_locomotion_backend),
            Node(
                condition=IfCondition(synthetic_imu),
                package="isaac_go2_nav2",
                executable="odom_to_imu",
                name="odom_to_imu",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "odom_topic": odom_topic,
                        "imu_topic": imu_topic,
                        "imu_frame": "imu_link",
                    }
                ],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            locomotion_backend,
                            "' in ",
                            str(sorted(POLICY_CONTROLLER_BACKENDS)),
                        ]
                    )
                ),
                package="isaac_go2_nav2",
                executable="go2_isaac_policy_controller",
                name="go2_isaac_policy_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "policy_path": policy_path,
                        "env_config_path": env_config_path,
                        "policy_runtime": LaunchConfiguration("policy_runtime"),
                        "strict_env_contract": LaunchConfiguration(
                            "policy_strict_env_contract"
                        ),
                        "imu_topic": imu_topic,
                        "joint_command_topic": joint_command_topic,
                        "action_scale": LaunchConfiguration("policy_action_scale"),
                        "action_clip": LaunchConfiguration("policy_action_clip"),
                        "safe_joint_clip": LaunchConfiguration("policy_safe_joint_clip"),
                        "target_blend": LaunchConfiguration("policy_target_blend"),
                        "target_step_limit": LaunchConfiguration(
                            "policy_target_step_limit"
                        ),
                        "linear_command_gain": LaunchConfiguration(
                            "policy_linear_command_gain"
                        ),
                        "angular_command_gain": LaunchConfiguration(
                            "policy_angular_command_gain"
                        ),
                        "max_policy_linear": LaunchConfiguration(
                            "policy_max_policy_linear"
                        ),
                        "max_policy_angular": LaunchConfiguration(
                            "policy_max_policy_angular"
                        ),
                        "command_deadband": LaunchConfiguration("policy_command_deadband"),
                        "policy_joint_order": LaunchConfiguration("policy_joint_order"),
                        "joint_command_order": LaunchConfiguration("joint_command_order"),
                        "run_policy_when_idle": LaunchConfiguration(
                            "policy_run_when_idle"
                        ),
                        "stand_when_state_stale": True,
                    }
                ],
            ),
            Node(
                condition=IfCondition(simulated_odom),
                package="isaac_go2_nav2",
                executable="cmd_vel_odom_simulator",
                name="cmd_vel_odom_simulator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "cmd_vel_topic": "/cmd_vel",
                        "odom_topic": odom_topic,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "publish_tf": False,
                        "pause_when_external_odom": True,
                    }
                ],
            ),
            Node(
                condition=IfCondition(publish_odom_tf),
                package="isaac_go2_nav2",
                executable="odom_tf_broadcaster",
                name="odom_tf_broadcaster",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "odom_topic": odom_topic,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "flatten_to_2d": LaunchConfiguration("flatten_odom_tf"),
                    }
                ],
            ),
            Node(
                condition=IfCondition(static_laser_tf),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=["0.24", "0", "0.12", "0", "0", "0", base_frame, laser_frame],
            ),
            Node(
                condition=IfCondition(synthetic_scan),
                package="isaac_go2_nav2",
                executable="map_scan_simulator",
                name="map_scan_simulator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_yaml": map_file,
                        "odom_topic": odom_topic,
                        "scan_topic": "/scan",
                        "frame_id": laser_frame,
                        "initial_x": -6.0,
                        "initial_y": -4.0,
                        "initial_yaw": 0.0,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_share / "launch" / "bringup_launch.py")),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                }.items(),
            ),
            Node(
                condition=IfCondition(start_target_follower),
                package="isaac_go2_nav2",
                executable="person_target_follower",
                name="person_target_follower",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "target_topic": target_topic,
                        "target_frame": target_frame,
                        "follow_distance": follow_distance,
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(nav2_share / "rviz" / "nav2_default_view.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
