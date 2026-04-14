"""Local FK/IK helpers for the ArmPi-FPV arm."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from .config import (
    ARM_JOINT_NAMES,
    BASE_HEIGHT_M,
    HOME_ARM_PULSES,
    IK_DAMPING,
    IK_MAX_ITERS,
    IK_ORIENTATION_TOLERANCE_RAD,
    IK_ORIENTATION_WEIGHT,
    IK_POSITION_TOLERANCE_M,
    IK_STEP_SCALE,
    JOINT_LIMITS_DEG,
    LINK1_M,
    LINK2_M,
    LINK3_M,
    PULSE_MAP_BY_NAME,
    TOOL_FRAME,
    TOOL_LINK_M,
)


def _as_np3(values: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"expected 3-vector, got shape {arr.shape}")
    return arr


def _as_np4(values: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float)
    if arr.shape != (4,):
        raise ValueError(f"expected quaternion [x, y, z, w], got shape {arr.shape}")
    return arr


def _normalize_quaternion(quat: Iterable[float]) -> np.ndarray:
    q = _as_np4(quat)
    n = np.linalg.norm(q)
    if n <= 0:
        raise ValueError("quaternion must have non-zero norm")
    return q / n


def _quat_xyzw_to_wxyz(quat: Iterable[float]) -> np.ndarray:
    q = _normalize_quaternion(quat)
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


def _quat_wxyz_to_xyzw(quat: Iterable[float]) -> np.ndarray:
    q = np.asarray(list(quat), dtype=float)
    return np.array([q[1], q[2], q[3], q[0]], dtype=float)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def _quat_inverse(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float) / np.dot(q, q)


def _quat_from_matrix(rot: np.ndarray) -> np.ndarray:
    m = rot
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z], dtype=float)


def _rotation_error_rad_vec(target_xyzw: Iterable[float], actual_xyzw: Iterable[float]) -> np.ndarray:
    target = _quat_xyzw_to_wxyz(target_xyzw)
    actual = _quat_xyzw_to_wxyz(actual_xyzw)
    dq = _quat_multiply(target, _quat_inverse(actual))
    if dq[0] < 0:
        dq = -dq
    xyz = dq[1:]
    norm_xyz = np.linalg.norm(xyz)
    if norm_xyz < 1e-12:
        return np.zeros(3, dtype=float)
    angle = 2.0 * math.atan2(norm_xyz, max(dq[0], 1e-12))
    axis = xyz / norm_xyz
    return axis * angle


def _rot_x(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _rot_z(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _translate(vec: Iterable[float]) -> np.ndarray:
    return _transform(np.eye(3), vec)


def _transform(rot: np.ndarray, trans: Iterable[float]) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = rot
    T[:3, 3] = np.asarray(list(trans), dtype=float)
    return T


def _mdh(alpha_prev: float, a_prev: float, theta: float, d: float = 0.0) -> np.ndarray:
    """Modified DH transform: Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)."""
    return (
        _transform(_rot_x(alpha_prev), [0.0, 0.0, 0.0]) @
        _translate([a_prev, 0.0, 0.0]) @
        _transform(_rot_z(theta), [0.0, 0.0, 0.0]) @
        _translate([0.0, 0.0, d])
    )


def _angle_transform(value: float, params: list[float], inverse: bool = False) -> float:
    if inverse:
        return ((value - params[5]) / (params[4] - params[3])) * (params[1] - params[0]) + params[2]
    return ((value - params[2]) / (params[1] - params[0])) * (params[4] - params[3]) + params[5]


JOINT_LIMITS_RAD = {
    name: tuple(math.radians(v) for v in limits) for name, limits in JOINT_LIMITS_DEG.items()
}


def home_arm_joints_rad() -> dict[str, float]:
    return {
        name: pulse_to_radians(name, HOME_ARM_PULSES[name])
        for name in ARM_JOINT_NAMES
    }


def clamp_joint_value(name: str, value_rad: float) -> float:
    lo, hi = JOINT_LIMITS_RAD[name]
    return min(max(value_rad, lo), hi)


def clamp_joint_dict(joints: dict[str, float]) -> dict[str, float]:
    return {name: clamp_joint_value(name, joints[name]) for name in ARM_JOINT_NAMES}


def pulse_to_radians(name: str, pulse: float) -> float:
    if name not in PULSE_MAP_BY_NAME:
        raise KeyError(f"no pulse mapping for joint '{name}'")
    return math.radians(_angle_transform(float(pulse), PULSE_MAP_BY_NAME[name], inverse=False))


def radians_to_pulse(name: str, radians_value: float) -> int:
    if name not in PULSE_MAP_BY_NAME:
        raise KeyError(f"no pulse mapping for joint '{name}'")
    deg = math.degrees(clamp_joint_value(name, radians_value))
    pulse = _angle_transform(deg, PULSE_MAP_BY_NAME[name], inverse=True)
    return int(round(min(max(pulse, 0.0), 1000.0)))


def arm_pulses_to_joints(pulses: dict[str, float]) -> dict[str, float]:
    return {name: pulse_to_radians(name, pulses[name]) for name in ARM_JOINT_NAMES}


def arm_joints_to_pulses(joints: dict[str, float]) -> dict[str, int]:
    return {name: radians_to_pulse(name, joints[name]) for name in ARM_JOINT_NAMES}


def coerce_joint_dict(joints: dict[str, float] | Iterable[float] | None) -> dict[str, float]:
    if joints is None:
        return home_arm_joints_rad()
    if isinstance(joints, dict):
        missing = [name for name in ARM_JOINT_NAMES if name not in joints]
        if missing:
            raise ValueError(f"missing joints: {missing}")
        return clamp_joint_dict({name: float(joints[name]) for name in ARM_JOINT_NAMES})
    values = list(joints)
    if len(values) != len(ARM_JOINT_NAMES):
        raise ValueError(f"expected {len(ARM_JOINT_NAMES)} joints, got {len(values)}")
    return clamp_joint_dict({name: float(value) for name, value in zip(ARM_JOINT_NAMES, values)})


def fk_matrix(joints: dict[str, float] | Iterable[float] | None = None) -> np.ndarray:
    q = coerce_joint_dict(joints)
    # Match the vendor transform.py structure: a Modified DH chain for the 5 arm
    # joints, plus an explicit final tool reach. The vendor comments indicate the
    # practical end-effector reach is modeled as link3 + tool_link.
    T = _translate([0.0, 0.0, BASE_HEIGHT_M])
    T = T @ _mdh(0.0, 0.0, q["base_yaw"], 0.0)
    T = T @ _mdh(-math.pi / 2.0, 0.0, q["shoulder"], 0.0)
    T = T @ _mdh(0.0, LINK1_M, q["elbow"], 0.0)
    T = T @ _mdh(0.0, LINK2_M, q["wrist_pitch"], 0.0)
    T = T @ _mdh(-math.pi / 2.0, 0.0, q["wrist_roll"], 0.0)
    T = T @ _translate([LINK3_M + TOOL_LINK_M, 0.0, 0.0])
    return T


def fk_pose(joints: dict[str, float] | Iterable[float] | None = None) -> dict:
    T = fk_matrix(joints)
    quat_xyzw = _quat_wxyz_to_xyzw(_quat_from_matrix(T[:3, :3]))
    return {
        "position": T[:3, 3].tolist(),
        "orientation": quat_xyzw.tolist(),
        "frame": TOOL_FRAME,
    }


def _orientation_distance_rad(target_xyzw: Iterable[float], actual_xyzw: Iterable[float]) -> float:
    return float(np.linalg.norm(_rotation_error_rad_vec(target_xyzw, actual_xyzw)))


def _ik_cost(
    joints_vector: np.ndarray,
    target_position: np.ndarray,
    target_orientation_xyzw: np.ndarray,
    orientation_weight: float,
) -> tuple[float, dict]:
    joints = {name: float(value) for name, value in zip(ARM_JOINT_NAMES, joints_vector)}
    pose = fk_pose(joints)
    position_error = target_position - np.asarray(pose["position"], dtype=float)
    orientation_error = _rotation_error_rad_vec(target_orientation_xyzw, pose["orientation"])
    cost = float(np.dot(position_error, position_error) + orientation_weight * np.dot(orientation_error, orientation_error))
    return cost, pose


def solve_ik(
    position: Iterable[float],
    orientation: Iterable[float],
    *,
    initial_joints: dict[str, float] | Iterable[float] | None = None,
    prefer_current: bool = True,
    allow_approx: bool = True,
    max_iters: int = IK_MAX_ITERS,
    damping: float = IK_DAMPING,
    step_scale: float = IK_STEP_SCALE,
    orientation_weight: float = IK_ORIENTATION_WEIGHT,
    position_tolerance_m: float = IK_POSITION_TOLERANCE_M,
    orientation_tolerance_rad: float = IK_ORIENTATION_TOLERANCE_RAD,
) -> dict:
    del prefer_current
    target_position = _as_np3(position)
    target_orientation = _normalize_quaternion(orientation)

    seed = coerce_joint_dict(initial_joints)
    q = np.array([seed[name] for name in ARM_JOINT_NAMES], dtype=float)
    lower = np.array([JOINT_LIMITS_RAD[name][0] for name in ARM_JOINT_NAMES], dtype=float)
    upper = np.array([JOINT_LIMITS_RAD[name][1] for name in ARM_JOINT_NAMES], dtype=float)

    best_cost, best_pose = _ik_cost(q, target_position, target_orientation, orientation_weight)
    best_q = q.copy()

    for _ in range(max_iters):
        current_pose = fk_pose({name: float(value) for name, value in zip(ARM_JOINT_NAMES, q)})
        pos_err = target_position - np.asarray(current_pose["position"], dtype=float)
        rot_err = _rotation_error_rad_vec(target_orientation, current_pose["orientation"])
        weighted_err = np.concatenate([pos_err, orientation_weight * rot_err])

        pos_norm = float(np.linalg.norm(pos_err))
        rot_norm = float(np.linalg.norm(rot_err))
        if pos_norm <= position_tolerance_m and rot_norm <= orientation_tolerance_rad:
            best_q = q.copy()
            best_pose = current_pose
            best_cost = float(np.dot(pos_err, pos_err) + orientation_weight * np.dot(rot_err, rot_err))
            break

        J = np.zeros((6, len(ARM_JOINT_NAMES)), dtype=float)
        eps = 1e-4
        for idx in range(len(ARM_JOINT_NAMES)):
            q_eps = q.copy()
            q_eps[idx] = min(max(q_eps[idx] + eps, lower[idx]), upper[idx])
            pose_eps = fk_pose({name: float(value) for name, value in zip(ARM_JOINT_NAMES, q_eps)})
            dpos = (np.asarray(pose_eps["position"], dtype=float) - np.asarray(current_pose["position"], dtype=float)) / eps
            drot = _rotation_error_rad_vec(pose_eps["orientation"], current_pose["orientation"]) / eps
            J[:3, idx] = dpos
            J[3:, idx] = orientation_weight * drot

        A = J @ J.T + (damping ** 2) * np.eye(6, dtype=float)
        dq = J.T @ np.linalg.solve(A, weighted_err)
        if not np.all(np.isfinite(dq)):
            break
        dq = np.clip(dq, -0.25, 0.25)
        q = np.clip(q + step_scale * dq, lower, upper)

        cost, pose = _ik_cost(q, target_position, target_orientation, orientation_weight)
        if cost < best_cost:
            best_cost = cost
            best_q = q.copy()
            best_pose = pose

    achieved_pose = best_pose
    achieved_position = np.asarray(achieved_pose["position"], dtype=float)
    position_error = float(np.linalg.norm(target_position - achieved_position))
    orientation_error = _orientation_distance_rad(target_orientation, achieved_pose["orientation"])
    success = position_error <= position_tolerance_m and orientation_error <= orientation_tolerance_rad

    if not success and not allow_approx:
        raise ValueError(
            "requested pose is not achievable within tolerance "
            f"(position_error={position_error:.4f} m, orientation_error={orientation_error:.4f} rad)"
        )

    joints = {name: float(value) for name, value in zip(ARM_JOINT_NAMES, best_q)}
    return {
        "success": success or allow_approx,
        "joints": joints,
        "pose_requested": {
            "position": target_position.tolist(),
            "orientation": target_orientation.tolist(),
            "frame": TOOL_FRAME,
        },
        "pose_achieved": achieved_pose,
        "position_error_m": position_error,
        "orientation_error_rad": orientation_error,
        "approximated": not success,
    }
