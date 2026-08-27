#!/usr/bin/env python3
"""Check the local Isaac Go2 policy contract before starting Nav2."""

from pathlib import Path

import numpy as np

from isaac_go2_nav2.go2_isaac_policy_controller import (
    EXPECTED_ACTION_SCALE,
    EXPECTED_DECIMATION,
    EXPECTED_DT,
    EXPECTED_OBS_SIZE,
    EXPECTED_STIFFNESS,
    EXPECTED_DAMPING,
    EXPECTED_EFFORT_LIMIT,
    EXPECTED_VELOCITY_LIMIT,
    IsaacGo2PolicyContract,
    NumpyTorchScriptMlp,
    default_env_config_path,
    default_policy_path,
    resolve_policy_joint_order,
)


def find_project_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "isaac_stage").exists() and (
            parent / "src" / "isaac_go2_nav2"
        ).exists():
            return parent
    return None


def check_stage(project_root):
    if project_root is None:
        return [], ["Could not locate project root for USD stage check"]

    stage_path = project_root / "isaac_stage" / "go2_nav2_low_config.usda"
    if not stage_path.exists():
        return [], [f"Stage not generated yet: {stage_path}"]

    text = stage_path.read_text(encoding="utf-8", errors="replace")
    errors = []
    warnings = []
    if "drive:angular:physics:stiffness = 45" in text:
        errors.append("Stage still contains old joint stiffness 45")
    if "drive:angular:physics:damping = 4.5" in text:
        errors.append("Stage still contains old joint damping 4.5")
    if text.count("drive:angular:physics:stiffness = 25") < 12:
        warnings.append("Stage does not show 12 authored stiffness=25 entries")
    damping_entries = text.count("drive:angular:physics:damping = 0.5")
    damping_entries += text.count("drive:angular:physics:damping = 2")
    if damping_entries < 12:
        warnings.append("Stage does not show 12 authored damping entries")
    if "physxScene:timeStepsPerSecond = 200" not in text:
        warnings.append("Stage does not author PhysX 200Hz timeStepsPerSecond")
    if "PhysxArticulationAPI" not in text:
        errors.append("Stage does not apply PhysxArticulationAPI to the Go2 articulation root")
    if "physxArticulation:solverPositionIterationCount = 4" not in text:
        errors.append("Stage does not author Go2 solverPositionIterationCount=4")
    if "physxArticulation:solverVelocityIterationCount = 0" not in text:
        errors.append("Stage does not author Go2 solverVelocityIterationCount=0")
    if "drive:angular:physics:maxForce = 45.43" in text:
        errors.append("Stage calf maxForce is 45.43, but the Isaac Go2 policy contract uses 23.5 for every joint")
    if "drive:angular:physics:maxForce = 23.7" in text:
        errors.append("Stage maxForce is 23.7, but the Isaac Go2 policy contract uses 23.5")
    if "ComputeOdom.outputs:linearVelocity" in text:
        errors.append(
            "Stage feeds local ComputeOdom.linearVelocity into ROS2 odometry; "
            "use globalLinearVelocity when publishRawVelocities=false"
        )
    if "ComputeOdom.outputs:globalLinearVelocity" not in text:
        warnings.append("Stage does not feed globalLinearVelocity into ROS2 odometry")
    return errors, warnings


def check_torch():
    try:
        import torch
    except Exception as exc:
        return f"torch unavailable, numpy fallback will run: {exc}"
    return f"torch available: {torch.__version__}"


def main():
    policy_joint_names = resolve_policy_joint_order("isaac_breadth")
    env_path = default_env_config_path()
    policy_path = default_policy_path()

    errors = []
    warnings = []

    try:
        contract = IsaacGo2PolicyContract(env_path, policy_joint_names)
        contract.validate_strict()
    except Exception as exc:
        errors.append(str(exc))
        contract = None

    try:
        policy = NumpyTorchScriptMlp(policy_path)
        action = policy.forward(np.zeros(EXPECTED_OBS_SIZE, dtype=np.float32))
        if action.shape != (12,):
            errors.append(f"Policy output shape expected (12,), got {action.shape}")
    except Exception as exc:
        errors.append(f"Policy file could not be loaded: {exc}")

    stage_errors, stage_warnings = check_stage(find_project_root())
    errors.extend(stage_errors)
    warnings.extend(stage_warnings)

    torch_status = check_torch()

    if errors:
        print("Isaac Go2 policy contract FAILED")
        for item in errors:
            print(f"- {item}")
        for item in warnings:
            print(f"warning: {item}")
        raise SystemExit(1)

    print("Isaac Go2 policy contract OK")
    print(f"- env: {env_path}")
    print(f"- policy: {policy_path}")
    print(
        "- timing: dt=%.3f, decimation=%d, policy_rate=50Hz"
        % (EXPECTED_DT, EXPECTED_DECIMATION)
    )
    print(
        "- actuators: stiffness=%.1f damping=%.1f effort=%.1f velocity=%.1f"
        % (
            EXPECTED_STIFFNESS,
            EXPECTED_DAMPING,
            EXPECTED_EFFORT_LIMIT,
            EXPECTED_VELOCITY_LIMIT,
        )
    )
    print(f"- action_scale: {EXPECTED_ACTION_SCALE}")
    print(
        "- obs[48]: base_lin_vel, base_ang_vel, projected_gravity, command, "
        "joint_pos_rel, joint_vel, last_action"
    )
    print(f"- {torch_status}")
    for item in warnings:
        print(f"warning: {item}")


if __name__ == "__main__":
    main()
