"""Offline kinematic audit for main_suture.m.

This mirrors the calibrated MATLAB FK and the hierarchical IK task split.
It checks the same entry, extraction and handoff TCP poses without starting
CoppeliaSim.  The report is diagnostic; MATLAB remains the runtime planner.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
REPORT = ROOT / "output" / "reports" / "suture_trajectory_validation.tsv"
AUDIT_SOLVER = "coupled"


def rotz4(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


def pose4(x, y, z, qx, qy, qz, qw) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], float)
    q /= np.linalg.norm(q)
    xq, yq, zq, wq = q
    r = np.array(
        [
            [1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * wq), 2 * (xq * zq + yq * wq)],
            [2 * (xq * yq + zq * wq), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * wq)],
            [2 * (xq * zq - yq * wq), 2 * (yq * zq + xq * wq), 1 - 2 * (xq * xq + yq * yq)],
        ]
    )
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, [x, y, z]
    return t


def fk(q: np.ndarray, arm: str) -> np.ndarray:
    base = np.eye(4)
    base[:3, 3] = ([0.6, 0, 0.51465] if arm == "R" else [0, 0, 0.51465])
    l010 = np.eye(4); l010[2, 3] = 0.0085
    l120 = np.array([[0, 0, -1, -0.0703], [0, 1, 0, 0], [1, 0, 0, 0.06605], [0, 0, 0, 1]], float)
    l230 = np.eye(4); l230[0, 3] = 0.4251
    l340 = np.eye(4); l340[0, 3] = 0.39215
    l450 = np.array([[0, 0, 1, 0.04557], [0, 1, 0, 0], [-1, 0, 0, 0.0397], [0, 0, 0, 1]], float)
    l560 = np.array([[0, 0, -1, 0.01443], [0, 1, 0, 0], [1, 0, 0, 0.04918], [0, 0, 0, 1]], float)
    l660 = np.eye(4); l660[2, 3] = 0.08938
    t6 = base @ l010 @ rotz4(q[0]) @ l120 @ rotz4(q[1]) @ l230 @ rotz4(q[2]) @ l340 @ rotz4(q[3]) @ l450 @ rotz4(q[4]) @ l560 @ rotz4(q[5]) @ l660
    if arm == "R":
        l670 = pose4(0.0442, -0.0682, 0.1175710689583, 0, 2**-0.5, 2**-0.5, 0)
        l7tool = pose4(0.0442, 0, 0, 0, 0, -2**-0.5, 2**-0.5)
    else:
        l670 = pose4(-0.0442, -0.0682, 0.1175710689583, -2**-0.5, 0, 0, 2**-0.5)
        l7tool = pose4(0.0442, 0, 0, 0, 0, 2**-0.5, 2**-0.5)
    m7 = np.eye(4); m7[2, 3] = q[6]
    ltool8 = pose4(0, 0, 0, 0, 0, -1, 0)
    l8link1 = pose4(0.000455975532532, -0.000129997730255, 0.226981401443, -2**-0.5, 2**-0.5, 0, 0)
    llink19 = pose4(0.000007484114705, 0.000230443676010, -0.225139383808, 2**-0.5, 0, 2**-0.5, 0)
    l9wrist = pose4(-0.003876566886900, -0.000133752822876, 0.000012136995792, -0.5, 0.5, 0.5, -0.5)
    lwristtip = pose4(-0.000006699866697, 0.000120966265910, 0.014496663479400, 0, 0, 0, 1)
    ttool = t6 @ l670 @ m7 @ l7tool
    return ttool @ ltool8 @ rotz4(q[7]) @ l8link1 @ llink19 @ rotz4(q[8]) @ l9wrist @ lwristtip


def rotation_vector(r: np.ndarray) -> np.ndarray:
    value = float(np.clip((np.trace(r) - 1) / 2, -1, 1))
    angle = math.acos(value)
    if angle < 1e-10:
        return np.zeros(3)
    if math.pi - angle < 1e-5:
        values, vectors = np.linalg.eig(r)
        axis = np.real(vectors[:, np.argmin(np.abs(values - 1))])
        axis /= np.linalg.norm(axis)
        return axis * angle
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * math.sin(angle))
    return axis * angle


def jacobian(q: np.ndarray, arm: str) -> np.ndarray:
    step = 1e-5
    current = fk(q, arm)
    j = np.zeros((6, 11))
    for index in range(9):
        changed = q.copy(); changed[index] += step
        shifted = fk(changed, arm)
        j[:3, index] = (shifted[:3, 3] - current[:3, 3]) / step
        j[3:, index] = rotation_vector(shifted[:3, :3] @ current[:3, :3].T) / step
    return j


def pinv_damped(a: np.ndarray, damping: float) -> np.ndarray:
    return a.T @ np.linalg.inv(a @ a.T + damping * damping * np.eye(a.shape[0]))


def solve_pose(target: np.ndarray, q_start: np.ndarray, arm: str):
    q = q_start.copy()
    p_indices = [0, 1, 2, 3, 4, 6]
    o_indices = [5, 7, 8]
    sp = np.eye(11)[:, p_indices]
    so = np.eye(11)[:, o_indices]
    for iteration in range(1, 301):
        current = fk(q, arm)
        ep = target[:3, 3] - current[:3, 3]
        eo = rotation_vector(target[:3, :3] @ current[:3, :3].T)
        if np.linalg.norm(ep) < 5e-4 and np.linalg.norm(eo) < 1e-3:
            return q, True, iteration, np.linalg.norm(ep), np.linalg.norm(eo)
        j = jacobian(q, arm)
        if AUDIT_SOLVER == "hierarchical":
            jp, jo = j[:3], j[3:]
            jpr = jp @ sp
            dqp = sp @ (pinv_damped(jpr, 0.01) @ ep)
            preserve = so - sp @ (pinv_damped(jpr, 0.001) @ (jp @ so))
            dqo = preserve @ (pinv_damped(jo @ preserve, 0.01) @ (eo - jo @ dqp))
            dq = dqp + dqo
        else:
            # Fallback reachability audit using every active joint together.
            active = list(range(9))
            ja = j[:, active]
            error = np.r_[ep, eo]
            dq = np.zeros(11)
            dq[active] = pinv_damped(ja, 0.006) @ error
        max_revolute = 0.12
        dq[:6] = np.clip(dq[:6], -max_revolute, max_revolute)
        dq[7:9] = np.clip(dq[7:9], -max_revolute, max_revolute)
        dq[6] = np.clip(dq[6], -0.004, 0.004)
        dq[9:] = 0
        q += 0.65 * dq
        q[:6] = np.clip(q[:6], -2 * math.pi, 2 * math.pi)
        q[6] = np.clip(q[6], 0, 0.16)
        q[7] = np.clip(q[7], -2 * math.pi, 2 * math.pi)
        q[8] = np.clip(q[8], math.radians(-85), math.radians(85))
    current = fk(q, arm)
    ep = np.linalg.norm(target[:3, 3] - current[:3, 3])
    eo = np.linalg.norm(rotation_vector(target[:3, :3] @ current[:3, :3].T))
    return q, False, 300, ep, eo


def interpolate(a: np.ndarray, b: np.ndarray, steps: int):
    # Small-segment rotation interpolation through the relative rotation log.
    relative = rotation_vector(b[:3, :3] @ a[:3, :3].T)
    for alpha in np.linspace(0, 1, steps):
        angle = np.linalg.norm(relative) * alpha
        if angle < 1e-12:
            rd = np.eye(3)
        else:
            axis = relative / np.linalg.norm(relative)
            k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            rd = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)
        t = np.eye(4)
        t[:3, :3] = rd @ a[:3, :3]
        t[:3, 3] = (1 - alpha) * a[:3, 3] + alpha * b[:3, 3]
        yield t


def ry(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def arc_poses(start: np.ndarray, center: np.ndarray, relation: np.ndarray, rotation: float, translation: np.ndarray, steps: int):
    needles, tcps = [], []
    for alpha in np.linspace(0, 1, steps):
        rd = ry(alpha * rotation)
        ta = max(0, (alpha - 0.55) / 0.45)
        needle = np.eye(4)
        needle[:3, :3] = rd @ start[:3, :3]
        needle[:3, 3] = center + rd @ (start[:3, 3] - center) + ta * translation
        needles.append(needle)
        tcps.append(needle @ np.linalg.inv(relation))
    return needles, tcps


def main():
    ql = np.array([-0.5235988, -0.7853982, 2.3561945, -2.2689280, -1.5707963, 1.3962634, 0.06, 0, 0, 0, 0])
    qr = np.array([3.6651914, 0.7853982, -2.3561945, 2.3561945, 1.5707963, 1.7453293, 0.1, 1.3962634, -0.8726646, 0.6981317, -0.6981317])
    left_live = pose4(-0.0965305180783, 0.463791318812, 0.737393945071, -0.442243035073, 0.825605445293, 0.0994321853465, 0.336020813566)
    right_live = pose4(0.0717914064246, 0.501164659681, 0.747866412348, -0.16093874618, 0.834979864671, 0.473866353381, 0.228818759514)
    calibration = {
        "L": left_live @ np.linalg.inv(fk(ql, "L")),
        "R": right_live @ np.linalg.inv(fk(qr, "R")),
    }
    print(f"initial_left_tcp={left_live[:3, 3].tolist()}")
    print(f"initial_right_tcp={right_live[:3, 3].tolist()}")
    print(f"left_model_to_scene_translation={calibration['L'][:3, 3].tolist()}")
    print(f"right_model_to_scene_translation={calibration['R'][:3, 3].tolist()}")
    t_tcp_r_needle = pose4(-1.27738568e-05, 0.00383664369, 0.00160058175, 0.640856385205, -0.640856378907, 0.298836231978, 0.298836245483)
    tip = np.array([0, 0.006, -0.0115])
    hand = np.array([0, 0, -0.01])
    circle_local = np.array([0, 0.005854474479272, -0.000125944725770])
    entry = np.array([-0.022, 0.345, 0.5905])
    exit_point = np.array([-0.038, 0.345, 0.5905])
    radial_local = tip - circle_local
    radius = np.linalg.norm(radial_local)
    half_chord = np.linalg.norm(entry - exit_point) / 2
    center = (entry + exit_point) / 2 + np.array([0, 0, math.sqrt(radius**2 - half_chord**2)])
    sweep = 2 * math.asin(half_chord / radius)
    lr = radial_local / radius
    lt = np.array([0, -lr[2], lr[1]]); lt /= np.linalg.norm(lt)
    ln = np.cross(lr, lt)
    wr = (entry - center) / radius
    # The mirrored needle-plane normal gives the right wrist a reachable
    # entry posture while preserving the same +Y tip sweep through tissue.
    wn = np.array([0, -1, 0])
    wt = np.cross(wn, wr); wt /= np.linalg.norm(wt)
    rw_n = np.column_stack((wr, wt, wn)) @ np.column_stack((lr, lt, ln)).T
    needle_entry = np.eye(4); needle_entry[:3, :3] = rw_n; needle_entry[:3, 3] = entry - rw_n @ tip
    needle_approach = needle_entry.copy(); needle_approach[:3, 3] += [0, 0, 0.035]
    tcp_r_approach = needle_approach @ np.linalg.inv(t_tcp_r_needle)
    tcp_r_entry = needle_entry @ np.linalg.inv(t_tcp_r_needle)
    print(f"right_approach_tcp={tcp_r_approach[:3, 3].tolist()}")
    print(f"right_entry_tcp={tcp_r_entry[:3, 3].tolist()}")
    insertion_needles, insertion_tcps = arc_poses(needle_entry, center, t_tcp_r_needle, sweep, np.zeros(3), 46)
    left_grasp_roll = math.radians(0)
    relation_left = np.eye(4)
    relation_left[:3, :3] = t_tcp_r_needle[:3, :3] @ rotz4(left_grasp_roll)[:3, :3]
    relation_left[:3, 3] = -relation_left[:3, :3] @ hand
    tcp_l_grasp = insertion_needles[-1] @ np.linalg.inv(relation_left)
    tcp_l_pre = tcp_l_grasp.copy(); tcp_l_pre[:3, 3] += [-0.025, 0, 0.015]

    phases = []
    def run_segment(name, arm, q, poses):
        max_ep = max_eo = 0.0
        for scene_target in poses:
            model_target = np.linalg.inv(calibration[arm]) @ scene_target
            q, ok, iterations, ep, eo = solve_pose(model_target, q, arm)
            max_ep, max_eo = max(max_ep, ep), max(max_eo, eo)
            if not ok:
                phases.append((name, arm, "FAIL", ep, eo, q[6], q[8], iterations))
                return q, False
        phases.append((name, arm, "PASS", max_ep, max_eo, q[6], q[8], iterations))
        return q, True

    qr, _ = run_segment("right_approach", "R", qr, interpolate(calibration["R"] @ fk(qr, "R"), tcp_r_approach, 18))
    qr, _ = run_segment("right_contact", "R", qr, interpolate(calibration["R"] @ fk(qr, "R"), tcp_r_entry, 10))
    qr, _ = run_segment("right_insert", "R", qr, insertion_tcps)
    ql, _ = run_segment("left_pregrasp", "L", ql, interpolate(calibration["L"] @ fk(ql, "L"), tcp_l_pre, 18))
    ql, _ = run_segment("left_grasp", "L", ql, interpolate(calibration["L"] @ fk(ql, "L"), tcp_l_grasp, 10))

    extraction_needles, extraction_tcps = arc_poses(insertion_needles[-1], center, relation_left, math.radians(85), np.array([-0.025, 0, 0.020]), 52)
    ql, _ = run_segment("left_extract", "L", ql, extraction_tcps)
    needle_handoff = extraction_needles[-1].copy(); needle_handoff[:3, 3] = [-0.030, 0.430, 0.690]
    tcp_l_handoff = needle_handoff @ np.linalg.inv(relation_left)
    ql, _ = run_segment("left_handoff", "L", ql, interpolate(calibration["L"] @ fk(ql, "L"), tcp_l_handoff, 20))
    tcp_r_grasp = needle_handoff @ np.linalg.inv(t_tcp_r_needle)
    tcp_r_pre = tcp_r_grasp.copy(); tcp_r_pre[:3, 3] += [0.025, 0, 0.015]
    qr, _ = run_segment("right_pre_handoff", "R", qr, interpolate(calibration["R"] @ fk(qr, "R"), tcp_r_pre, 20))
    qr, _ = run_segment("right_regrasp", "R", qr, interpolate(calibration["R"] @ fk(qr, "R"), tcp_r_grasp, 10))

    REPORT.parent.mkdir(parents=True, exist_ok=True)
    with REPORT.open("w", encoding="utf-8", newline="") as handle:
        handle.write("phase\tarm\tresult\tposition_error_mm\torientation_error_deg\tjoint7_mm\tjoint9_deg\titerations_last\n")
        for row in phases:
            handle.write(f"{row[0]}\t{row[1]}\t{row[2]}\t{row[3]*1000:.4f}\t{math.degrees(row[4]):.4f}\t{row[5]*1000:.3f}\t{math.degrees(row[6]):.3f}\t{row[7]}\n")
    print(f"radius_mm={radius*1000:.4f}")
    print(f"sweep_deg={math.degrees(sweep):.4f}")
    print(f"center={center.tolist()}")
    for row in phases:
        print(row[0], row[2], f"ep={row[3]*1000:.3f}mm", f"eo={math.degrees(row[4]):.3f}deg", f"q7={row[5]*1000:.1f}mm")
    print(REPORT)
    raise SystemExit(1 if any(row[2] != "PASS" for row in phases) else 0)


if __name__ == "__main__":
    main()
