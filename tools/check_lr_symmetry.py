#!/usr/bin/env python3
"""Check that the two hands' configs behave as mirrors, without hardware.

Builds a synthetic hand posture, mirrors it, feeds each version to the matching
hand's config and compares the results. Because the inputs are exact mirrors,
**the outputs must be mirrors too.** A mismatch means something in the config,
the URDF or the MANO conversion is left/right asymmetric.

    .venv/bin/python tools/check_lr_symmetry.py      # exit code 0 on success

Two sequences are run:

    1. pinch   the thumb is brought to the index finger. **Pass/fail is judged
               here** -- this is the range where pinch projection is active, so
               it maps directly to real-world performance.
    2. fist    a free posture where the thumb does nothing. **Informational.**
               A difference of roughly 15 mm is normal, because the left and
               right joint limit widths differ.

There are two pass criteria:

    left/right difference of the index, middle and ring fingertips
        the kinematics are an exact mirror, so these must line up
    left/right difference in minimum pinch distance
        this is the core teleoperation function

**The thumb tip's absolute position is not part of the judgement.** It is normal
for the joint_13_0 limit width to differ by 39 degrees between hands, so the
thumb reaches the same pinch through a slightly different posture. As long as
the pinch distance matches there is no functional problem (measured: 0.0 mm
difference).

The two URDFs are an exact kinematic mirror (confirmed 2026-08-31). The joint
correspondence is:

    index, middle, ring   only abduction (joint_0/4/8_0) flips sign;
                          the three flexion joints stay as they are
    thumb                 joint_12/14/15_0 flip sign, joint_13_0 is 180deg - theta

joint_13_0 follows a different rule because the two URDFs' zero points are
180 degrees apart. A posture that logs as 134 degrees on the left reads as
46 degrees on the right.
"""
from __future__ import annotations

import argparse

import numpy as np

from v5s_teleop.hand.mano import raw_nodes_to_mano21, to_mano_frame
from v5s_teleop.retarget.builder import build_retargeting

#: Fingertip links. Both URDFs share these names.
TIPS = {"thumb": "link_15_0_tip", "index": "link_3_0_tip",
        "middle": "link_7_0_tip", "ring": "link_11_0_tip"}

#: Left angle to right angle. Joints that only flip sign.
_NEGATED = ("joint_0_0", "joint_4_0", "joint_8_0",
            "joint_12_0", "joint_14_0", "joint_15_0")

#: (lateral x, knuckle y, three segment lengths) -- rough adult hand, in metres.
_FINGERS = {
    "index":  (+0.022, 0.092, (0.040, 0.025, 0.020)),
    "middle": (-0.001, 0.097, (0.045, 0.028, 0.021)),
    "ring":   (-0.023, 0.092, (0.042, 0.026, 0.020)),
    "little": (-0.043, 0.080, (0.033, 0.019, 0.018)),
}
_BASE = {"index": 5, "middle": 10, "ring": 15, "little": 20}

#: Flipping x in the glove's raw frame turns left into right.
MIRROR = np.array([-1.0, 1.0, 1.0])

_INDEX_TIP, _THUMB_TIP = 9, 4      # raw node indices


def synth_left_hand(curl: float, thumb: float) -> np.ndarray:
    """(25, 3) left-hand raw skeleton. `curl` and `thumb` run 0 (open) to 1 (closed).

    This is not a real hand but a **reference posture for left/right comparison**.
    Do not read anything into the absolute values -- this check only looks at the
    difference between hands.
    """
    n = np.zeros((25, 3))
    a = thumb * 1.1
    p = np.array([0.018, 0.020, -0.004])
    n[1] = p
    for k, seg in enumerate((0.035, 0.032, 0.025)):
        d = np.array([np.cos(a * (0.5 + 0.3 * k)) * 0.55,
                      np.cos(a * (0.5 + 0.3 * k)) * 0.75,
                      -np.sin(a * (0.6 + 0.4 * k))])
        p = p + seg * d / np.linalg.norm(d)
        n[2 + k] = p
    for f, (x, y, segs) in _FINGERS.items():
        i = _BASE[f]
        n[i] = (x, y * 0.45, 0.0)
        p = np.array([x, y, 0.0])
        n[i + 1] = p
        ang = 0.0
        for k, seg in enumerate(segs):
            ang += curl * (1.7, 1.6, 1.1)[k]
            p = p + seg * np.array([0.0, np.cos(ang), -np.sin(ang)])
            n[i + 2 + k] = p
    return n


def pinch_left_hand(gap_m: float, curl: float = 0.25) -> np.ndarray:
    """A left-hand posture with the thumb tip `gap_m` away from the index tip.

    The whole thumb chain (nodes 1-4) is translated as a rigid body, which
    preserves the segment lengths. The MANO frame is defined only by the wrist
    and the index and middle knuckles, so it is unaffected.
    """
    n = synth_left_hand(curl, 0.35)
    v = n[_THUMB_TIP] - n[_INDEX_TIP]
    want = n[_INDEX_TIP] + v / np.linalg.norm(v) * gap_m
    n[1:5] += want - n[_THUMB_TIP]
    return n


def mirror_joints(q: np.ndarray, names: list[str]) -> np.ndarray:
    """Left joint angles to the corresponding right joint angles."""
    out = q.copy()
    for i, name in enumerate(names):
        if name in _NEGATED:
            out[i] = -q[i]
        elif name == "joint_13_0":
            out[i] = np.pi - q[i]
    return out


def _run(config: str, is_right: bool, poses):
    """Retarget a list of postures, returning (angles, tip positions, joint names)."""
    ret = build_retargeting(config)
    opt, robot = ret.optimizer, ret.optimizer.robot
    wrist = robot.get_link_index("virtual_wrist")
    tips = {k: robot.get_link_index(v) for k, v in TIPS.items()}
    origin_idx, task_idx = opt.target_link_human_indices

    qs, ps = [], []
    for raw in poses:
        mano = to_mano_frame(raw_nodes_to_mano21(raw), is_right=is_right)
        opt.set_human_keypoints(mano)
        q = ret.retarget(mano[task_idx] - mano[origin_idx])
        qs.append(q)
        robot.compute_forward_kinematics(q)
        w = robot.get_link_pose(wrist)
        ps.append({k: w[:3, :3].T @ (robot.get_link_pose(i)[:3, 3] - w[:3, 3])
                   for k, i in tips.items()})
    return np.array(qs), ps, list(robot.dof_joint_names)


_FLIP = np.array([1.0, -1.0, 1.0])


def _compare(left_poses, args):
    """Run both hands, returning (per-tip difference in mm, left pinch distance,
    right pinch distance)."""
    right_poses = [p * MIRROR for p in left_poses]
    qL, pL, names = _run(args.left, False, left_poses)
    qR, pR, _ = _run(args.right, True, right_poses)
    diff = {k: np.array([np.linalg.norm(a[k] - b[k] * _FLIP)
                         for a, b in zip(pL, pR)]) * 1e3 for k in TIPS}
    pin = lambda P: np.array(
        [np.linalg.norm(a["thumb"] - a["index"]) for a in P]) * 1e3
    return diff, pin(pL), pin(pR), qL, qR, names


def _table(title, diff, pinL, pinR):
    print(f"\n── {title} " + "─" * max(0, 46 - len(title)))
    print(f"{'tip':<7}{'mean L/R difference':>22}{'max':>10}")
    for k in TIPS:
        print(f"{k:<6}{diff[k].mean():14.1f} mm{diff[k].max():7.1f} mm")
    print(f"min pinch distance   left {pinL.min():.1f} mm   right {pinR.min():.1f} mm"
          f"   ({pinR.min() - pinL.min():+.1f} mm)")
    return max(d.max() for d in diff.values())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--left", default="src/v5s_teleop/configs/v5s_left_dexpilot.yml")
    ap.add_argument("--right", default="src/v5s_teleop/configs/v5s_right_dexpilot.yml")
    ap.add_argument("--frames", type=int, default=25)
    ap.add_argument("--tol-mm", type=float, default=5.0,
                    help="tolerance for the index/middle/ring fingertip difference")
    ap.add_argument("--pinch-tol-mm", type=float, default=3.0,
                    help="tolerance for the minimum pinch distance difference")
    args = ap.parse_args()

    gaps = np.concatenate([np.linspace(0.10, 0.004, args.frames),
                           np.linspace(0.004, 0.10, args.frames // 2)])
    d1, l1, r1, q1, q2, names = _compare([pinch_left_hand(g) for g in gaps], args)
    _table(f"1. pinch -- judged ({len(gaps)} frames, gap 100 to 4 mm)", d1, l1, r1)
    fingers = max(d1[k].max() for k in ("index", "middle", "ring"))
    pinch = abs(r1.min() - l1.min())

    close = np.linspace(0.0, 1.0, args.frames)
    seq = list(close) + list(close[::-1])
    d2, l2, r2, *_ = _compare([synth_left_hand(c, c) for c in seq], args)
    _table(f"2. fist -- informational ({len(seq)} frames)", d2, l2, r2)
    print("  ^ difference while the thumb idles. Around 15 mm is normal, because "
          "the joint limit widths differ.")

    err = np.degrees(np.abs(q2 - np.array([mirror_joints(q, names) for q in q1])))
    bad = [(names[i], err[:, i].max()) for i in range(len(names)) if err[:, i].max() > 5.0]
    if bad:
        print("\njoints whose mirrored angle differs by more than 5 deg during the pinch:")
        for n, e in sorted(bad, key=lambda t: -t[1]):
            print(f"  {n:<12} max {e:6.1f} deg")

    ok_f = fingers <= args.tol_mm
    ok_p = pinch <= args.pinch_tol_mm
    print()
    print(f"  {'OK  ' if ok_f else 'FAIL'}  index/middle/ring fingertip difference "
          f"{fingers:.1f} mm  (tolerance {args.tol_mm:.1f})")
    print(f"  {'OK  ' if ok_p else 'FAIL'}  minimum pinch distance difference "
          f"{pinch:.1f} mm  (tolerance {args.pinch_tol_mm:.1f})")
    print(f"  note  thumb tip difference {d1['thumb'].max():.1f} mm -- not judged "
          f"(joint_13_0 limit width differs; this is normal)")
    ok = ok_f and ok_p
    print(f"\n{'PASSED' if ok else 'FAILED'}")
    if not ok:
        print("Suspect a left/right asymmetry in the config, the URDF or the MANO "
              "conversion. See the joint_limit_overrides comment in the "
              "right-hand config.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
