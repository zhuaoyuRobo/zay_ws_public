#!/usr/bin/env python3
"""rl_sar/himloco Go2 policy adapter for Isaac Sim."""

import threading
import time
import zipfile
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


TRAINING_JOINT_NAMES = [
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

RL_SAR_RAW_JOINT_NAMES = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]

JOINT_ORDERS = {
    "training": TRAINING_JOINT_NAMES,
    "rl_sar_raw_config": RL_SAR_RAW_JOINT_NAMES,
}

DEFAULT_Q_BY_JOINT = {
    "FL_hip_joint": 0.10,
    "FL_thigh_joint": 0.80,
    "FL_calf_joint": -1.50,
    "FR_hip_joint": -0.10,
    "FR_thigh_joint": 0.80,
    "FR_calf_joint": -1.50,
    "RL_hip_joint": 0.10,
    "RL_thigh_joint": 1.00,
    "RL_calf_joint": -1.50,
    "RR_hip_joint": -0.10,
    "RR_thigh_joint": 1.00,
    "RR_calf_joint": -1.50,
}

ACTION_SCALE_BY_JOINT = {
    "FL_hip_joint": 0.125,
    "FL_thigh_joint": 0.25,
    "FL_calf_joint": 0.25,
    "FR_hip_joint": 0.125,
    "FR_thigh_joint": 0.25,
    "FR_calf_joint": 0.25,
    "RL_hip_joint": 0.125,
    "RL_thigh_joint": 0.25,
    "RL_calf_joint": 0.25,
    "RR_hip_joint": 0.125,
    "RR_thigh_joint": 0.25,
    "RR_calf_joint": 0.25,
}

SAFE_LIMITS_BY_JOINT = {
    "FL_hip_joint": (-0.95, 0.95),
    "FL_thigh_joint": (-1.25, 3.20),
    "FL_calf_joint": (-2.65, -0.85),
    "FR_hip_joint": (-0.95, 0.95),
    "FR_thigh_joint": (-1.25, 3.20),
    "FR_calf_joint": (-2.65, -0.85),
    "RL_hip_joint": (-0.95, 0.95),
    "RL_thigh_joint": (-0.25, 4.20),
    "RL_calf_joint": (-2.65, -0.85),
    "RR_hip_joint": (-0.95, 0.95),
    "RR_thigh_joint": (-0.25, 4.20),
    "RR_calf_joint": (-2.65, -0.85),
}


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_joint_name(name):
    text = str(name).replace("\\", "/")
    text = text.split("/")[-1]
    text = text.split("::")[-1]
    return text


def default_policy_path():
    try:
        share = Path(get_package_share_directory("isaac_go2_nav2"))
        return share / "assets" / "rl_sar_go2_himloco" / "himloco.pt"
    except Exception:
        return (
            Path(__file__).resolve().parents[1]
            / "assets"
            / "rl_sar_go2_himloco"
            / "himloco.pt"
        )


def resolve_joint_order(order):
    key = str(order).strip()
    if key in JOINT_ORDERS:
        return list(JOINT_ORDERS[key])

    names = [name.strip() for name in key.split(",") if name.strip()]
    if len(names) == 12 and set(names) == set(TRAINING_JOINT_NAMES):
        return names

    valid = ", ".join(sorted(JOINT_ORDERS))
    raise ValueError(
        "policy_joint_order must be one of [%s] or a comma-separated "
        "permutation of the 12 Go2 joint names" % valid
    )


def normalize_quat_xyzw(quat):
    q = np.asarray(quat, dtype=np.float32)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-6 or not np.isfinite(norm):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return q / norm


def quat_to_matrix_xyzw(quat):
    x, y, z, w = normalize_quat_xyzw(quat)
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


def world_to_body(vec, quat_xyzw):
    return quat_to_matrix_xyzw(quat_xyzw).T @ np.asarray(vec, dtype=np.float32)


def elu(value):
    result = np.asarray(value, dtype=np.float32).copy()
    negative = result <= 0.0
    result[negative] = np.expm1(result[negative])
    return result


class HimLocoPolicy:
    """Numpy runtime for rl_sar's exported himloco TorchScript model."""

    def __init__(self, path):
        self.path = Path(path)
        self.estimator_weights = []
        self.estimator_biases = []
        self.actor_weights = []
        self.actor_biases = []
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
        if not self.path.exists():
            raise FileNotFoundError(self.path)

        with zipfile.ZipFile(self.path) as archive:
            actor_w0 = self._load_tensor(archive, 0, 512 * 64).reshape(512, 64)
            actor_b0 = self._load_tensor(archive, 1, 512)
            actor_w1 = self._load_tensor(archive, 2, 256 * 512).reshape(256, 512)
            actor_b1 = self._load_tensor(archive, 3, 256)
            actor_w2 = self._load_tensor(archive, 4, 128 * 256).reshape(128, 256)
            actor_b2 = self._load_tensor(archive, 5, 128)
            actor_w3 = self._load_tensor(archive, 6, 12 * 128).reshape(12, 128)
            actor_b3 = self._load_tensor(archive, 7, 12)

            est_w0 = self._load_tensor(archive, 8, 128 * 270).reshape(128, 270)
            est_b0 = self._load_tensor(archive, 9, 128)
            est_w1 = self._load_tensor(archive, 10, 64 * 128).reshape(64, 128)
            est_b1 = self._load_tensor(archive, 11, 64)
            est_w2 = self._load_tensor(archive, 12, 19 * 64).reshape(19, 64)
            est_b2 = self._load_tensor(archive, 13, 19)

        self.actor_weights = [actor_w0, actor_w1, actor_w2, actor_w3]
        self.actor_biases = [actor_b0, actor_b1, actor_b2, actor_b3]
        self.estimator_weights = [est_w0, est_w1, est_w2]
        self.estimator_biases = [est_b0, est_b1, est_b2]

    def _forward_mlp(self, x, weights, biases):
        for weight, bias in zip(weights[:-1], biases[:-1]):
            x = elu(weight @ x + bias)
        return weights[-1] @ x + biases[-1]

    def forward(self, obs_history):
        history = np.asarray(obs_history, dtype=np.float32)
        if history.shape != (270,):
            raise ValueError(f"himloco obs_history must be shape (270,), got {history.shape}")

        estimator_out = self._forward_mlp(
            history,
            self.estimator_weights,
            self.estimator_biases,
        )
        estimated_velocity = estimator_out[:3]
        latent = estimator_out[3:]
        latent_norm = max(float(np.linalg.norm(latent)), 1.0e-12)
        latent = latent / latent_norm

        actor_input = np.concatenate([history[:45], estimated_velocity, latent]).astype(
            np.float32
        )
        return self._forward_mlp(actor_input, self.actor_weights, self.actor_biases).astype(
            np.float32
        )


class Go2RlSarIsaacController(Node):
    """Bridge Nav2 velocity commands into rl_sar/himloco joint targets."""

    def __init__(self):
        super().__init__("go2_rl_sar_isaac_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("policy_path", str(default_policy_path()))
        self.declare_parameter("policy_joint_order", "training")
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("policy_rate", 50.0)
        self.declare_parameter("stand_duration", 1.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("command_deadband", 0.02)
        self.declare_parameter("linear_command_gain", 1.4)
        self.declare_parameter("angular_command_gain", 0.7)
        self.declare_parameter("max_policy_linear", 0.50)
        self.declare_parameter("max_policy_angular", 0.50)
        self.declare_parameter("action_clip", 10.0)
        self.declare_parameter("target_blend", 0.85)
        self.declare_parameter("target_step_limit", 0.08)
        self.declare_parameter("run_policy_when_idle", False)

        self.policy = HimLocoPolicy(self.get_parameter("policy_path").value)
        self.policy_joint_names = resolve_joint_order(
            self.get_parameter("policy_joint_order").value
        )
        self.default_q_policy = np.array(
            [DEFAULT_Q_BY_JOINT[name] for name in self.policy_joint_names],
            dtype=np.float32,
        )
        self.action_scale_policy = np.array(
            [ACTION_SCALE_BY_JOINT[name] for name in self.policy_joint_names],
            dtype=np.float32,
        )
        self.safe_low_policy = np.array(
            [SAFE_LIMITS_BY_JOINT[name][0] for name in self.policy_joint_names],
            dtype=np.float32,
        )
        self.safe_high_policy = np.array(
            [SAFE_LIMITS_BY_JOINT[name][1] for name in self.policy_joint_names],
            dtype=np.float32,
        )
        self.policy_to_ros_indices = np.array(
            [self.policy_joint_names.index(name) for name in TRAINING_JOINT_NAMES],
            dtype=np.int64,
        )

        self.lock = threading.Lock()
        self.last_cmd = np.zeros(3, dtype=np.float32)
        self.last_cmd_time = 0.0
        self.last_imu_time = 0.0
        self.last_joint_time = 0.0
        self.imu_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.imu_angular = np.zeros(3, dtype=np.float32)
        self.joint_pos_policy = self.default_q_policy.copy()
        self.joint_vel_policy = np.zeros(12, dtype=np.float32)
        self.last_action = np.zeros(12, dtype=np.float32)
        self.last_target_policy = self.default_q_policy.copy()
        self.obs_history = [np.zeros(45, dtype=np.float32) for _ in range(6)]
        self.next_policy_time = 0.0
        self.start_time = time.monotonic()
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
            Imu,
            self.get_parameter("imu_topic").value,
            self.on_imu,
            20,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self.on_joint_state,
            20,
        )
        self.create_timer(
            1.0 / float(self.get_parameter("control_rate").value),
            self.on_timer,
        )
        self.get_logger().info(
            "Loaded rl_sar/himloco policy %s; policy_joint_order=%s; output=%s"
            % (
                self.get_parameter("policy_path").value,
                ",".join(self.policy_joint_names),
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

    def on_imu(self, msg):
        with self.lock:
            self.imu_quat = normalize_quat_xyzw(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]
            )
            self.imu_angular = np.array(
                [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                dtype=np.float32,
            )
            self.last_imu_time = time.monotonic()

    def on_joint_state(self, msg):
        name_to_msg_index = {
            normalize_joint_name(name): index for index, name in enumerate(msg.name)
        }
        with self.lock:
            updated = False
            for index, name in enumerate(self.policy_joint_names):
                msg_index = name_to_msg_index.get(name)
                if msg_index is None:
                    continue
                if msg_index < len(msg.position):
                    self.joint_pos_policy[index] = msg.position[msg_index]
                    updated = True
                if msg_index < len(msg.velocity):
                    self.joint_vel_policy[index] = msg.velocity[msg_index]
            if updated:
                self.last_joint_time = time.monotonic()

    def scaled_command(self, now):
        timeout = float(self.get_parameter("command_timeout").value)
        deadband = float(self.get_parameter("command_deadband").value)
        run_idle = bool(self.get_parameter("run_policy_when_idle").value)
        max_linear = float(self.get_parameter("max_policy_linear").value)
        max_angular = float(self.get_parameter("max_policy_angular").value)

        with self.lock:
            raw_cmd = self.last_cmd.copy()
            recent = now - self.last_cmd_time <= timeout

        nonzero = float(np.max(np.abs(raw_cmd))) >= deadband
        command_active = recent and nonzero
        if not command_active and not run_idle:
            raw_cmd = np.zeros(3, dtype=np.float32)

        return (
            np.array(
                [
                    clamp(
                        float(raw_cmd[0])
                        * float(self.get_parameter("linear_command_gain").value),
                        -max_linear,
                        max_linear,
                    ),
                    clamp(
                        float(raw_cmd[1])
                        * float(self.get_parameter("linear_command_gain").value),
                        -max_linear,
                        max_linear,
                    ),
                    clamp(
                        float(raw_cmd[2])
                        * float(self.get_parameter("angular_command_gain").value),
                        -max_angular,
                        max_angular,
                    ),
                ],
                dtype=np.float32,
            ),
            command_active,
        )

    def build_observation(self, now):
        state_timeout = float(self.get_parameter("state_timeout").value)
        with self.lock:
            imu_fresh = now - self.last_imu_time <= state_timeout
            joint_fresh = now - self.last_joint_time <= state_timeout
            quat = (
                self.imu_quat.copy()
                if imu_fresh
                else np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            )
            ang_vel = self.imu_angular.copy() if imu_fresh else np.zeros(3, dtype=np.float32)
            joint_pos = self.joint_pos_policy.copy()
            joint_vel = self.joint_vel_policy.copy()
            last_action = self.last_action.copy()

        command, command_active = self.scaled_command(now)
        projected_gravity = world_to_body(np.array([0.0, 0.0, -1.0], dtype=np.float32), quat)

        obs = np.concatenate(
            [
                command * np.array([2.0, 2.0, 0.25], dtype=np.float32),
                ang_vel * 0.25,
                projected_gravity,
                (joint_pos - self.default_q_policy),
                joint_vel * 0.05,
                last_action,
            ]
        ).astype(np.float32)
        return obs, command, command_active, imu_fresh, joint_fresh

    def run_policy_if_due(self, now, obs):
        policy_period = 1.0 / float(self.get_parameter("policy_rate").value)
        if now < self.next_policy_time:
            return self.last_action.copy(), self.last_target_policy.copy()

        self.next_policy_time = now + policy_period
        self.obs_history.insert(0, np.clip(obs, -100.0, 100.0).copy())
        del self.obs_history[6:]
        obs_history = np.concatenate(self.obs_history).astype(np.float32)

        action = self.policy.forward(obs_history)
        if not np.all(np.isfinite(action)):
            self.get_logger().warn("rl_sar policy output was non-finite; holding stand target")
            action = np.zeros(12, dtype=np.float32)
        action = np.clip(
            action,
            -float(self.get_parameter("action_clip").value),
            float(self.get_parameter("action_clip").value),
        ).astype(np.float32)
        target = self.default_q_policy + self.action_scale_policy * action
        target = np.clip(target, self.safe_low_policy, self.safe_high_policy).astype(
            np.float32
        )

        blend = clamp(float(self.get_parameter("target_blend").value), 0.0, 1.0)
        target = (blend * target + (1.0 - blend) * self.last_target_policy).astype(
            np.float32
        )
        step_limit = float(self.get_parameter("target_step_limit").value)
        if step_limit > 0.0:
            target = np.clip(
                target,
                self.last_target_policy - step_limit,
                self.last_target_policy + step_limit,
            ).astype(np.float32)

        self.last_action = action.copy()
        self.last_target_policy = target.copy()
        return action, target

    def publish_joint_command(self, target_policy):
        target_ros_order = target_policy[self.policy_to_ros_indices]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(TRAINING_JOINT_NAMES)
        msg.position = [float(value) for value in target_ros_order]
        self.joint_command_pub.publish(msg)

    def hold_stand(self):
        self.last_action = np.zeros(12, dtype=np.float32)
        self.last_target_policy = self.default_q_policy.copy()
        self.publish_joint_command(self.last_target_policy)

    def on_timer(self):
        now = time.monotonic()
        obs, command, command_active, imu_fresh, joint_fresh = self.build_observation(now)

        standing = now - self.start_time < float(self.get_parameter("stand_duration").value)
        stale = not imu_fresh or not joint_fresh
        if standing or stale or not command_active:
            self.hold_stand()
            action = np.zeros(12, dtype=np.float32)
            target = self.default_q_policy
            mode = "stand" if standing else "hold"
        else:
            action, target = self.run_policy_if_due(now, obs)
            self.publish_joint_command(target)
            mode = "policy"

        if now >= self.last_log:
            self.get_logger().info(
                "%s cmd=(%.2f %.2f %.2f) action_norm=%.2f target0=%.2f "
                "imu=%s joint=%s"
                % (
                    mode,
                    command[0],
                    command[1],
                    command[2],
                    float(np.linalg.norm(action)),
                    target[0],
                    "ok" if imu_fresh else "stale",
                    "ok" if joint_fresh else "stale",
                )
            )
            self.last_log = now + 2.0


def main():
    rclpy.init()
    node = Go2RlSarIsaacController()
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
