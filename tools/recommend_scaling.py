"""Recommend a scaling_factor from the MANUS calibration measurements.

The wrist offset (x, y, z) is a fixed value derived from the robot URDF's
workspace shape and its reach in the flexion direction, so it does not depend
on hand size (settled 2026-08-24; see the config file). scaling_factor, by
contrast, is "how many times longer the robot's fingers are than the user's",
so it has to change when the user does. This tool computes only that ratio.

Method: measure the MCP-to-fingertip length of the index, middle and ring
        fingers on both robot and human, take robot / human per finger, and
        average. The thumb is excluded -- the robot thumb is an opposed design
        with a completely different joint layout, so its length is not
        "measured the same way".

Usage:
    .venv/bin/python tools/recommend_scaling.py
    .venv/bin/python tools/recommend_scaling.py --glove-id 0x1EC9928C
    .venv/bin/python tools/recommend_scaling.py --urdf urdf/v5_sense/allegro_hand_description_right.urdf
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from dex_retargeting.robot_wrapper import RobotWrapper

DEFAULT_CALIB = (
    Path.home() / "Manus" / "Core 3" / "CoreLite.Settings.3.1.1.Calibrations.json"
)
DEFAULT_CALIB_ALT = Path.home() / ".config" / "Manus" / "Core 3" / "CoreLite.Settings.3.1.1.Calibrations.json"

# Robot side (MCP link, fingertip tip link). The names are shared by both hands.
ROBOT_FINGERS: dict[str, tuple[str, str]] = {
    "index": ("link_0_0", "link_3_0_tip"),
    "middle": ("link_4_0", "link_7_0_tip"),
    "ring": ("link_8_0", "link_11_0_tip"),
}
# Human side (MANUS measurements.fingerMeasurements indices:
#             0=index, 1=middle, 2=ring, 3=little)
HUMAN_FINGER_IDX = {"index": 0, "middle": 1, "ring": 2}


def load_calibration(path: Path, glove_id: str | None) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"calibration file not found: {path}")
    text = path.read_text()
    data = json.loads(text[text.index("{"):])
    profiles = data.get("gloveProfiles") or {}
    if not profiles:
        raise ValueError(f"{path} has no stored glove profile -- calibrate first")

    if glove_id is not None:
        key = glove_id if glove_id in profiles else glove_id.upper()
        if key not in profiles:
            raise ValueError(
                f"no profile for glove ID {glove_id}. "
                f"stored profiles: {list(profiles.keys())}"
            )
        return profiles[key]

    if len(profiles) > 1:
        raise ValueError(
            f"{len(profiles)} profiles are stored ({list(profiles.keys())}). "
            f"Use --glove-id to choose one."
        )
    return next(iter(profiles.values()))


def robot_finger_length(urdf_path: str, mcp_link: str, tip_link: str) -> float:
    robot = RobotWrapper(urdf_path)
    robot.compute_forward_kinematics(np.zeros(robot.dof))
    mcp = robot.get_link_pose(robot.get_link_index(mcp_link))[:3, 3]
    tip = robot.get_link_pose(robot.get_link_index(tip_link))[:3, 3]
    return float(np.linalg.norm(tip - mcp) * 1000.0)  # m -> mm


def human_finger_length(profile: dict, finger: str) -> float:
    m = profile["measurements"]["fingerMeasurements"][HUMAN_FINGER_IDX[finger]]
    return (
        m["proximalLength"] + m["intermediateLength"] + m["distalLength"]
    ) * 1000.0  # m -> mm


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calibration", type=Path, default=None,
                    help="path to the MANUS calibration JSON (default: search the standard locations)")
    ap.add_argument("--glove-id", default=None,
                    help="hexadecimal ID of the glove profile to use (e.g. 0x1EC9928C). "
                         "Optional when only one profile is stored")
    ap.add_argument("--urdf", default="urdf/v5_sense/allegro_hand_description_left.urdf",
                    help="path to the robot URDF (default: left hand)")
    args = ap.parse_args()

    calib_path = args.calibration
    if calib_path is None:
        calib_path = DEFAULT_CALIB_ALT if DEFAULT_CALIB_ALT.exists() else DEFAULT_CALIB

    try:
        profile = load_calibration(calib_path, args.glove_id)
    except (FileNotFoundError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    print(f"calibration: {calib_path}")
    print(f"glove side: {profile.get('side', '?')}\n")

    print(f"{'finger':8}{'robot(mm)':>12}{'human(mm)':>12}{'ratio':>10}")
    print("-" * 42)
    ratios = []
    for finger, (mcp, tip) in ROBOT_FINGERS.items():
        r = robot_finger_length(args.urdf, mcp, tip)
        h = human_finger_length(profile, finger)
        ratio = r / h
        ratios.append(ratio)
        print(f"{finger:8}{r:12.1f}{h:12.1f}{ratio:10.3f}")

    recommended = float(np.mean(ratios))
    spread = float(np.std(ratios))
    print("-" * 42)
    print(f"{'mean':8}{'':12}{'':12}{recommended:10.3f}")
    print()
    print(f"recommended scaling_factor = {recommended:.2f}  (spread across fingers +/-{spread:.3f})")
    print()
    print("write it into the config file:")
    print(f'  scaling_factor: {recommended:.2f}')
    print()
    print("or apply it live while hardware runs:")
    print(f'  ros2 param set /v5s_teleop scaling_factor {recommended:.2f}')
    print()
    print("NOTE: wrist_offset has nothing to do with this calculation -- it is a")
    print("      fixed value derived from the robot workspace and is not changed.")
    print("      See the wrist_offset comment in the config file for the reasoning.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
