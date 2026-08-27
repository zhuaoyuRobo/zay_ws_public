#!/usr/bin/env python3
"""ROS bridge for the NVIDIA Isaac Go2 flat-terrain policy.

This node keeps Nav2 as the high-level planner:

    /cmd_vel -> official 48-D Go2 policy observation -> 12 joint targets -> /joint_command

The observation, action scale, timing, default joint pose, and PD contract are
taken from the Isaac/IsaacLab Go2 ``physx_env.yaml`` used with
``physx_policy.pt``. The legacy Gazebo/rl_sar path is intentionally not part of
this controller.
"""

import fnmatch
import math
import threading
import time
import zipfile
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


ROS_JOINT_NAMES = [
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
]

ISAAC_BREADTH_JOINT_NAMES = [
    "FL_hip_joint",
    "FR_hip_joint",
    "RL_hip_joint",
    "RR_hip_joint",
    "FL_thigh_joint",
    "FR_thigh_joint",
    "RL_thigh_joint",
    "RR_thigh_joint",
    "FL_calf_joint",
    "FR_calf_joint",
    "RL_calf_joint",
    "RR_calf_joint",
]

POLICY_JOINT_ORDERS = {
    "isaac_breadth": ISAAC_BREADTH_JOINT_NAMES,
    "training": ISAAC_BREADTH_JOINT_NAMES,
    "ros_leg": ROS_JOINT_NAMES,
}

EXPECTED_DT = 0.005
EXPECTED_DECIMATION = 4
EXPECTED_ACTION_SCALE = 0.25
EXPECTED_STIFFNESS = 25.0
EXPECTED_DAMPING = 0.5
EXPECTED_EFFORT_LIMIT = 23.5
EXPECTED_VELOCITY_LIMIT = 30.0
EXPECTED_OBS_SIZE = 48
EXPECTED_ACTION_SIZE = 12

FALLBACK_DEFAULT_Q = np.array(
    [
        0.10,
        0.80,
        -1.50,
        -0.10,
        0.80,
        -1.50,
        0.10,
        1.00,
        -1.50,
        -0.10,
        1.00,
        -1.50,
    ],
    dtype=np.float32,
)

SAFE_LOW = np.array(
    [
        -0.95,
        -1.25,
        -2.65,
        -0.95,
        -1.25,
        -2.65,
        -0.95,
        -0.25,
        -2.65,
        -0.95,
        -0.25,
        -2.65,
    ],
    dtype=np.float32,
)

SAFE_HIGH = np.array(
    [
        0.95,
        3.20,
        -0.85,
        0.95,
        3.20,
        -0.85,
        0.95,
        4.20,
        -0.85,
        0.95,
        4.20,
        -0.85,
    ],
    dtype=np.float32,
)


def clamp(value, low, high):
    return max(low, min(high, value))


def default_policy_path():
    return ""


def default_env_config_path():
    return ""


def normalize_joint_name(name):
    text = str(name).replace("\\", "/")
    text = text.split("/")[-1]
    text = text.split("::")[-1]
    return text


def normalize_quat(quat):
    q = np.asarray(quat, dtype=np.float32)
    norm = float(np.linalg.norm(q))
    if norm < 1e-6 or not np.isfinite(norm):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return q / norm


def quat_to_matrix(quat):
    x, y, z, w = normalize_quat(quat)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


def world_to_body(vec, quat):
    return quat_to_matrix(quat).T @ np.asarray(vec, dtype=np.float32)


def elu(value):
    result = np.asarray(value, dtype=np.float32).copy()
    negative = result <= 0.0
    result[negative] = np.expm1(result[negative])
    return result


class SafeLoaderIgnoreUnknown(yaml.SafeLoader):
    def ignore_unknown(self, node):
        return None

    def tuple_constructor(self, node):
        return tuple(self.construct_sequence(node))


SafeLoaderIgnoreUnknown.add_constructor(
    "tag:yaml.org,2002:python/tuple",
    SafeLoaderIgnoreUnknown.tuple_constructor,
)
SafeLoaderIgnoreUnknown.add_constructor(None, SafeLoaderIgnoreUnknown.ignore_unknown)


def load_env_config(path):
    with Path(path).open("rb") as stream:
        return yaml.load(stream, Loader=SafeLoaderIgnoreUnknown)


def _pattern_matches(name, pattern):
    return fnmatch.fnmatch(name, pattern.replace(".", "*") + "*")


def _expand_by_patterns(mapping, joint_names, default=0.0):
    values = []
    for joint in joint_names:
        found = False
        if isinstance(mapping, dict):
            for pattern, value in mapping.items():
                if _pattern_matches(joint, pattern):
                    values.append(float(value))
                    found = True
                    break
        elif isinstance(mapping, (float, int)):
            values.append(float(mapping))
            found = True
        if not found:
            values.append(float(default))
    return np.asarray(values, dtype=np.float32)


def _expand_actuator_value(actuator, key, joint_names, default=0.0):
    exprs = actuator.get("joint_names_expr", [])
    raw = actuator.get(key, default)
    if isinstance(raw, dict):
        patterns = raw
    else:
        patterns = {expr: raw for expr in exprs}
    return _expand_by_patterns(patterns, joint_names, default)


class IsaacGo2PolicyContract:
    """Parsed subset of the Isaac Go2 env config used by the policy wrapper."""

    def __init__(self, path, policy_joint_names):
        self.path = Path(path)
        self.raw = load_env_config(self.path)
        self.policy_joint_names = list(policy_joint_names)
        self.dt = float(self.raw["sim"]["dt"])
        self.decimation = int(self.raw["decimation"])
        self.render_interval = int(self.raw["sim"].get("render_interval", self.decimation))

        action_cfg = self.raw.get("actions", {}).get("joint_pos", {})
        self.action_scale = float(
            self.raw.get("action_scale", action_cfg.get("scale", EXPECTED_ACTION_SCALE))
        )

        robot_cfg = self.raw["scene"]["robot"]
        init_state = robot_cfg["init_state"]
        self.default_pos = _expand_by_patterns(
            init_state.get("joint_pos", {}),
            self.policy_joint_names,
            0.0,
        )
        self.default_vel = _expand_by_patterns(
            init_state.get("joint_vel", {}),
            self.policy_joint_names,
            0.0,
        )

        actuators = robot_cfg["actuators"]
        if len(actuators) != 1:
            raise ValueError(
                "Expected exactly one Go2 actuator group in env config, got %d"
                % len(actuators)
            )
        actuator = next(iter(actuators.values()))
        self.control_mode = str(actuator.get("control_mode", "position"))
        self.stiffness = _expand_actuator_value(
            actuator, "stiffness", self.policy_joint_names, 0.0
        )
        self.damping = _expand_actuator_value(
            actuator, "damping", self.policy_joint_names, 0.0
        )
        self.effort_limit = _expand_actuator_value(
            actuator, "effort_limit", self.policy_joint_names, 0.0
        )
        self.velocity_limit = _expand_actuator_value(
            actuator, "velocity_limit", self.policy_joint_names, 0.0
        )
        self.friction = _expand_actuator_value(
            actuator, "friction", self.policy_joint_names, 0.0
        )

    @property
    def policy_rate(self):
        return 1.0 / (self.dt * float(self.decimation))

    def validate_strict(self):
        errors = []
        _expect_close(errors, "sim.dt", self.dt, EXPECTED_DT)
        _expect_equal(errors, "decimation", self.decimation, EXPECTED_DECIMATION)
        _expect_close(errors, "action scale", self.action_scale, EXPECTED_ACTION_SCALE)
        _expect_all_close(
            errors, "stiffness", self.stiffness, EXPECTED_STIFFNESS
        )
        _expect_all_close(errors, "damping", self.damping, EXPECTED_DAMPING)
        _expect_all_close(
            errors, "effort limit", self.effort_limit, EXPECTED_EFFORT_LIMIT
        )
        _expect_all_close(
            errors, "velocity limit", self.velocity_limit, EXPECTED_VELOCITY_LIMIT
        )
        _expect_all_close(errors, "default velocity", self.default_vel, 0.0)
        if self.control_mode != "position":
            errors.append("control_mode expected position, got %s" % self.control_mode)
        if len(self.default_pos) != EXPECTED_ACTION_SIZE:
            errors.append("default joint position count expected 12")
        if errors:
            raise ValueError("Isaac Go2 env contract mismatch:\n- " + "\n- ".join(errors))


def _expect_close(errors, name, actual, expected, tol=1e-6):
    if abs(float(actual) - float(expected)) > tol:
        errors.append("%s expected %.6g, got %.6g" % (name, expected, actual))


def _expect_equal(errors, name, actual, expected):
    if actual != expected:
        errors.append("%s expected %s, got %s" % (name, expected, actual))


def _expect_all_close(errors, name, values, expected, tol=1e-6):
    arr = np.asarray(values, dtype=np.float32)
    if arr.size != EXPECTED_ACTION_SIZE or not np.allclose(arr, expected, atol=tol):
        errors.append("%s expected all %.6g, got %s" % (name, expected, arr.tolist()))


class TorchScriptPolicy:
    runtime_name = "torch"

    def __init__(self, path):
        try:
            import torch
        except Exception as exc:
            raise ImportError("torch is not installed for python3") from exc
        self.torch = torch
        self.path = Path(path)
        if not self.path.is_file():
            raise FileNotFoundError(self.path)
        self.policy = torch.jit.load(str(self.path), map_location="cpu")
        self.policy.eval()

    def forward(self, obs):
        tensor = self.torch.as_tensor(np.asarray(obs, dtype=np.float32))
        with self.torch.no_grad():
            action = self.policy(tensor).detach().cpu().numpy().reshape(-1)
        if action.shape != (EXPECTED_ACTION_SIZE,):
            raise ValueError(
                "policy action must be shape (12,), got %s" % (action.shape,)
            )
        return action.astype(np.float32)


class NumpyTorchScriptMlp:
    """Small numpy runtime for Isaac Lab's exported Go2 MLP policy."""

    runtime_name = "numpy_torchscript_mlp"

    def __init__(self, path):
        self.path = Path(path)
        self.weights = []
        self.biases = []
        self._load()

    def _load_tensor(self, archive, index, expected_size):
        candidates = [
            name
            for name in archive.namelist()
            if name.endswith(f"/data/{index}") or name == f"data/{index}"
        ]
        if not candidates:
            raise ValueError(f"tensor data/{index} not found in {self.path}")
        data = np.frombuffer(archive.read(candidates[0]), dtype="<f4").copy()
        if data.size != expected_size:
            raise ValueError(
                f"tensor data/{index} has {data.size} values, expected {expected_size}"
            )
        return data

    def _load(self):
        if not self.path.is_file():
            raise FileNotFoundError(self.path)

        with zipfile.ZipFile(self.path) as archive:
            w0 = self._load_tensor(archive, 0, 128 * EXPECTED_OBS_SIZE).reshape(
                128, EXPECTED_OBS_SIZE
            )
            b0 = self._load_tensor(archive, 1, 128)
            w1 = self._load_tensor(archive, 2, 128 * 128).reshape(128, 128)
            b1 = self._load_tensor(archive, 3, 128)
            w2 = self._load_tensor(archive, 4, 128 * 128).reshape(128, 128)
            b2 = self._load_tensor(archive, 5, 128)
            w3 = self._load_tensor(archive, 6, EXPECTED_ACTION_SIZE * 128).reshape(
                EXPECTED_ACTION_SIZE, 128
            )
            b3 = self._load_tensor(archive, 7, EXPECTED_ACTION_SIZE)

        self.weights = [w0, w1, w2, w3]
        self.biases = [b0, b1, b2, b3]

    def forward(self, obs):
        x = np.asarray(obs, dtype=np.float32)
        if x.shape != (EXPECTED_OBS_SIZE,):
            raise ValueError(f"policy obs must be shape (48,), got {x.shape}")
        for weight, bias in zip(self.weights[:-1], self.biases[:-1]):
            x = elu(weight @ x + bias)
        return (self.weights[-1] @ x + self.biases[-1]).astype(np.float32)


def load_policy_runtime(path, runtime):
    runtime_key = str(runtime).strip().lower()
    if runtime_key in ("auto", "torch"):
        try:
            return TorchScriptPolicy(path)
        except Exception:
            if runtime_key == "torch":
                raise
    if runtime_key in ("auto", "numpy", "numpy_torchscript_mlp"):
        return NumpyTorchScriptMlp(path)
    raise ValueError("policy_runtime must be auto, torch, or numpy")


def resolve_policy_joint_order(order):
    key = str(order).strip()
    if key in POLICY_JOINT_ORDERS:
        return list(POLICY_JOINT_ORDERS[key])

    names = [normalize_joint_name(name.strip()) for name in key.split(",") if name.strip()]
    if len(names) != len(ROS_JOINT_NAMES) or set(names) != set(ROS_JOINT_NAMES):
        valid = ", ".join(sorted(POLICY_JOINT_ORDERS))
        raise ValueError(
            "policy_joint_order must be one of [%s] or a comma-separated "
            "permutation of the 12 Go2 joint names" % valid
        )
    return names


class Go2IsaacPolicyController(Node):
    """Bridge Nav2 velocity commands into Isaac Go2 joint position targets."""

    def __init__(self):
        super().__init__("go2_isaac_policy_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("policy_path", str(default_policy_path()))
        self.declare_parameter("env_config_path", str(default_env_config_path()))
        self.declare_parameter("policy_runtime", "auto")
        self.declare_parameter("strict_env_contract", True)
        self.declare_parameter("rate", 0.0)
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("action_scale", 0.0)
        self.declare_parameter("action_clip", 0.0)
        self.declare_parameter("safe_joint_clip", False)
        self.declare_parameter("target_blend", 1.0)
        self.declare_parameter("target_step_limit", 0.0)
        self.declare_parameter("linear_command_gain", 1.0)
        self.declare_parameter("angular_command_gain", 1.0)
        self.declare_parameter("max_policy_linear", 0.0)
        self.declare_parameter("max_policy_angular", 0.0)
        self.declare_parameter("command_deadband", 0.03)
        self.declare_parameter("odom_twist_in_world", False)
        self.declare_parameter("require_odom", True)
        self.declare_parameter("run_policy_when_idle", False)
        self.declare_parameter("policy_joint_order", "isaac_breadth")
        self.declare_parameter("joint_command_order", "isaac_breadth")
        self.declare_parameter("stand_when_state_stale", True)

        self.policy_joint_names = resolve_policy_joint_order(
            self.get_parameter("policy_joint_order").value
        )
        self.ros_to_policy_indices = np.array(
            [ROS_JOINT_NAMES.index(name) for name in self.policy_joint_names],
            dtype=np.int64,
        )
        self.policy_to_ros_indices = np.array(
            [self.policy_joint_names.index(name) for name in ROS_JOINT_NAMES],
            dtype=np.int64,
        )
        self.output_joint_names = resolve_policy_joint_order(
            self.get_parameter("joint_command_order").value
        )
        self.ros_to_output_indices = np.array(
            [ROS_JOINT_NAMES.index(name) for name in self.output_joint_names],
            dtype=np.int64,
        )

        self.contract = IsaacGo2PolicyContract(
            self.get_parameter("env_config_path").value,
            self.policy_joint_names,
        )
        if bool(self.get_parameter("strict_env_contract").value):
            self.contract.validate_strict()

        self.action_scale = self.contract.action_scale
        action_scale_override = float(self.get_parameter("action_scale").value)
        if action_scale_override > 0.0:
            self.action_scale = action_scale_override
            self.get_logger().warn(
                "Overriding env action scale %.3f with %.3f; this is not the "
                "strict Isaac Go2 contract"
                % (self.contract.action_scale, self.action_scale)
            )

        self.default_q_policy = self.contract.default_pos.astype(np.float32)
        self.default_vel_policy = self.contract.default_vel.astype(np.float32)
        self.default_q_ros = self.default_q_policy[self.policy_to_ros_indices]
        self.safe_low_policy = SAFE_LOW[self.ros_to_policy_indices]
        self.safe_high_policy = SAFE_HIGH[self.ros_to_policy_indices]

        self.policy = load_policy_runtime(
            self.get_parameter("policy_path").value,
            self.get_parameter("policy_runtime").value,
        )

        self.lock = threading.Lock()
        self.last_cmd = np.zeros(3, dtype=np.float32)
        self.last_cmd_time = 0.0
        self.last_odom_time = 0.0
        self.last_joint_time = 0.0
        self.odom_linear = np.zeros(3, dtype=np.float32)
        self.odom_angular = np.zeros(3, dtype=np.float32)
        self.odom_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.joint_pos = self.default_q_ros.copy()
        self.joint_vel = np.zeros(EXPECTED_ACTION_SIZE, dtype=np.float32)
        self.last_action = np.zeros(EXPECTED_ACTION_SIZE, dtype=np.float32)
        self.last_target = self.default_q_ros.copy()
        self.last_log = 0.0

        self.joint_command_pub = self.create_publisher(
            JointState,
            self.get_parameter("joint_command_topic").value,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.on_cmd,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.on_odom,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self.on_joint_state,
            20,
        )

        rate_param = float(self.get_parameter("rate").value)
        self.policy_rate = rate_param if rate_param > 0.0 else self.contract.policy_rate
        self.create_timer(1.0 / self.policy_rate, self.on_timer)

        self.get_logger().info(
            "Loaded Isaac Go2 policy %s using %s; env=%s; rate=%.2f Hz; "
            "dt=%.4f decimation=%d action_scale=%.2f; policy_order=%s; "
            "command_order=%s; output=%s"
            % (
                self.get_parameter("policy_path").value,
                self.policy.runtime_name,
                self.get_parameter("env_config_path").value,
                self.policy_rate,
                self.contract.dt,
                self.contract.decimation,
                self.action_scale,
                ",".join(self.policy_joint_names),
                ",".join(self.output_joint_names),
                self.get_parameter("joint_command_topic").value,
            )
        )

    def on_cmd(self, msg):
        with self.lock:
            self.last_cmd = np.array(
                [msg.linear.x, msg.linear.y, msg.angular.z],
                dtype=np.float32,
            )
            self.last_cmd_time = time.monotonic()

    def on_odom(self, msg):
        with self.lock:
            self.odom_linear = np.array(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ],
                dtype=np.float32,
            )
            self.odom_angular = np.array(
                [
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z,
                ],
                dtype=np.float32,
            )
            self.odom_quat = normalize_quat(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ]
            )
            self.last_odom_time = time.monotonic()

    def on_joint_state(self, msg):
        name_to_msg_index = {
            normalize_joint_name(name): index for index, name in enumerate(msg.name)
        }
        with self.lock:
            updated = False
            for index, name in enumerate(ROS_JOINT_NAMES):
                msg_index = name_to_msg_index.get(name)
                if msg_index is None:
                    continue
                if msg_index < len(msg.position):
                    self.joint_pos[index] = msg.position[msg_index]
                    updated = True
                if msg_index < len(msg.velocity):
                    self.joint_vel[index] = msg.velocity[msg_index]
            if updated:
                self.last_joint_time = time.monotonic()

    def scaled_command(self, cmd):
        command = np.array(
            [
                float(cmd[0]) * float(self.get_parameter("linear_command_gain").value),
                float(cmd[1]) * float(self.get_parameter("linear_command_gain").value),
                float(cmd[2]) * float(self.get_parameter("angular_command_gain").value),
            ],
            dtype=np.float32,
        )
        max_linear = float(self.get_parameter("max_policy_linear").value)
        max_angular = float(self.get_parameter("max_policy_angular").value)
        if max_linear > 0.0:
            command[0] = clamp(command[0], -max_linear, max_linear)
            command[1] = clamp(command[1], -max_linear, max_linear)
        if max_angular > 0.0:
            command[2] = clamp(command[2], -max_angular, max_angular)
        return command

    def build_observation(self, now):
        state_timeout = float(self.get_parameter("state_timeout").value)
        command_timeout = float(self.get_parameter("timeout").value)

        with self.lock:
            raw_cmd = self.last_cmd.copy()
            command_recent = (
                self.last_cmd_time > 0.0 and now - self.last_cmd_time <= command_timeout
            )
            command_deadband = float(self.get_parameter("command_deadband").value)
            command_nonzero = float(np.max(np.abs(raw_cmd))) > command_deadband
            command_active = command_recent and command_nonzero
            run_idle = bool(self.get_parameter("run_policy_when_idle").value)
            cmd = raw_cmd if command_recent or run_idle else np.zeros(3, dtype=np.float32)

            odom_fresh = self.last_odom_time > 0.0 and now - self.last_odom_time <= state_timeout
            joint_fresh = (
                self.last_joint_time > 0.0 and now - self.last_joint_time <= state_timeout
            )

            quat = self.odom_quat.copy()
            base_lin_vel = self.odom_linear.copy() if odom_fresh else np.zeros(3, dtype=np.float32)
            base_ang_vel = self.odom_angular.copy() if odom_fresh else np.zeros(3, dtype=np.float32)
            joint_pos = self.joint_pos.copy()[self.ros_to_policy_indices]
            joint_vel = self.joint_vel.copy()[self.ros_to_policy_indices]
            last_action = self.last_action.copy()

        if bool(self.get_parameter("odom_twist_in_world").value):
            base_lin_vel = world_to_body(base_lin_vel, quat)
            base_ang_vel = world_to_body(base_ang_vel, quat)
        projected_gravity = world_to_body(np.array([0.0, 0.0, -1.0], dtype=np.float32), quat)
        velocity_command = self.scaled_command(cmd)

        obs = np.concatenate(
            [
                base_lin_vel,
                base_ang_vel,
                projected_gravity,
                velocity_command,
                joint_pos - self.default_q_policy,
                joint_vel - self.default_vel_policy,
                last_action,
            ]
        ).astype(np.float32)
        if obs.shape != (EXPECTED_OBS_SIZE,):
            raise RuntimeError("Isaac Go2 observation shape mismatch: %s" % (obs.shape,))
        return obs, command_active, odom_fresh, joint_fresh, velocity_command

    def publish_joint_command(self, target):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.output_joint_names)
        msg.position = [float(value) for value in target[self.ros_to_output_indices]]
        self.joint_command_pub.publish(msg)

    def on_timer(self):
        now = time.monotonic()
        obs, command_active, odom_fresh, joint_fresh, velocity_command = (
            self.build_observation(now)
        )

        run_idle = bool(self.get_parameter("run_policy_when_idle").value)
        stand_if_stale = bool(self.get_parameter("stand_when_state_stale").value)
        require_odom = bool(self.get_parameter("require_odom").value)
        stale_state = not joint_fresh or (require_odom and not odom_fresh)

        if not command_active and not run_idle:
            target = self.default_q_ros.copy()
            action = np.zeros(EXPECTED_ACTION_SIZE, dtype=np.float32)
        elif stand_if_stale and stale_state:
            target = self.default_q_ros.copy()
            action = np.zeros(EXPECTED_ACTION_SIZE, dtype=np.float32)
        else:
            action = self.policy.forward(obs).astype(np.float32)
            if not np.all(np.isfinite(action)):
                self.get_logger().warn("Policy output was non-finite; holding stand target")
                action = np.zeros(EXPECTED_ACTION_SIZE, dtype=np.float32)
            action_clip = float(self.get_parameter("action_clip").value)
            if action_clip > 0.0:
                action = np.clip(action, -action_clip, action_clip)
            target_policy = self.default_q_policy + self.action_scale * action
            if bool(self.get_parameter("safe_joint_clip").value):
                target_policy = np.clip(
                    target_policy,
                    self.safe_low_policy,
                    self.safe_high_policy,
                )
            target = target_policy[self.policy_to_ros_indices]

        blend = clamp(float(self.get_parameter("target_blend").value), 0.0, 1.0)
        target = (blend * target + (1.0 - blend) * self.last_target).astype(np.float32)
        step_limit = float(self.get_parameter("target_step_limit").value)
        if step_limit > 0.0:
            target = np.clip(
                target,
                self.last_target - step_limit,
                self.last_target + step_limit,
            ).astype(np.float32)
        self.last_action = action.astype(np.float32)
        self.last_target = target.copy()
        self.publish_joint_command(target)

        if now >= self.last_log:
            self.get_logger().info(
                "cmd=(%.2f %.2f %.2f) action_norm=%.2f target0=%.2f "
                "odom=%s joint=%s runtime=%s"
                % (
                    velocity_command[0],
                    velocity_command[1],
                    velocity_command[2],
                    float(np.linalg.norm(action)),
                    target[0],
                    "ok" if odom_fresh else "stale",
                    "ok" if joint_fresh else "stale",
                    self.policy.runtime_name,
                )
            )
            self.last_log = now + 2.0


def main():
    rclpy.init()
    node = Go2IsaacPolicyController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
