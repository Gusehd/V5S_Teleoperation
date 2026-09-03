"""Live check of glove to retargeting (no hardware needed).

Takes glove frames from the bridge, solves robot joint angles with DexPilot and
displays the result. **It does not move hardware** -- it exists to eyeball
whether the config values are right (project rule: check the output joint
angles in the log before running a new config on hardware).

    .venv/bin/python tools/retarget_live.py [--config ...] [--scaling 0.91]

Live controls (single keypress, no Enter):
    + / -   scaling_factor by 0.05
    r       reset the accumulated statistics
    q       quit
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

import v5s_teleop
from v5s_teleop.bridge.zmq_client import GloveSubscriber
from v5s_teleop.retarget.builder import RetargetingSpec, build_retargeting

FINGERS = (("thumb", "link_15_0_tip"), ("index", "link_3_0_tip"),
           ("middle", "link_7_0_tip"), ("ring", "link_11_0_tip"))
# Human-side MANO indices (thumb tip, index tip, middle tip, ring tip)
HUMAN_TIP = (4, 8, 12, 16)


class RawKeys:
    """Read a single keypress without Enter. Does nothing when not a terminal."""

    def __enter__(self):
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        if self.fd is not None:
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def get(self) -> str | None:
        if self.fd is None:
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def __exit__(self, *exc):
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None, help="retargeting config yml")
    ap.add_argument("--scaling", type=float, default=None, help="override scaling_factor")
    ap.add_argument("--endpoint", default="tcp://127.0.0.1:5555")
    args = ap.parse_args()

    cfg = Path(args.config) if args.config else (
        Path(v5s_teleop.__file__).parent / "configs/v5s_left_dexpilot.yml")
    spec = RetargetingSpec.from_yaml(cfg)
    if args.scaling is not None:
        spec.scaling_factor = args.scaling

    print(f"config : {cfg.name}")
    print(f"URDF : {Path(spec.urdf_path).name}")
    print(f"wrist ref : {spec.wrist_link_name}   eta1 : {spec.eta1 * 1000:.0f} mm")
    print("building...")
    ret = build_retargeting(spec)
    opt = ret.optimizer
    robot = opt.robot
    joints = list(robot.dof_joint_names)  # same order as the retarget() output
    tip_idx = {n: robot.get_link_index(l) for n, l in FINGERS}
    ref_o, ref_t = opt.target_link_human_indices

    # Keep the limits in the same (pinocchio) order as the output q.
    limits = np.array(robot.joint_limits)
    print("waiting for a connection...\n")

    with GloveSubscriber(args.endpoint) as sub, RawKeys() as keys:
        n = 0
        t0 = time.time()
        last_draw = 0.0
        solve_ms = 0.0
        while True:
            k = keys.get()
            if k:
                if k == "q":
                    break
                if k in "+-":
                    opt.scaling = max(0.05, opt.scaling + (0.05 if k == "+" else -0.05))
                elif k == "r":
                    n = 0
                    t0 = time.time()

            frame = sub.recv(timeout_ms=200)
            if frame is None:
                sys.stdout.write("\033[H\033[Jno frames from the bridge "
                                 "-- is manus_bridge running?\n")
                sys.stdout.flush()
                continue

            mano = frame.to_mano21()
            ref = mano[ref_t] - mano[ref_o]

            t1 = time.perf_counter()
            q = ret.retarget(ref)
            solve_ms = 0.9 * solve_ms + 0.1 * (time.perf_counter() - t1) * 1000
            n += 1

            now = time.time()
            if now - last_draw < 0.08:
                continue
            last_draw = now

            # SeqRetargeting.retarget() returns values in robot.dof_joint_names
            # (pinocchio) order (seq_retarget.py: robot_qpos[idx_pin2target] =
            # qpos). So it can go straight into FK -- no reordering here.
            robot.compute_forward_kinematics(q)
            rtip = {nm: robot.get_link_pose(i)[:3, 3] for nm, i in tip_idx.items()}

            L = [
                (f"scaling {opt.scaling:.2f}   eta1 {opt.eta1 * 1000:.0f}mm   "
                 f"{n / (now - t0):.0f} Hz received   solve {solve_ms:.1f} ms"),
                "",
                "  thumb-to-finger tip distance [mm]   human     robot   (pinch target 27mm)",
            ]
            for fi, (nm, _) in enumerate(FINGERS[1:], start=1):
                h = np.linalg.norm(mano[HUMAN_TIP[fi]] - mano[HUMAN_TIP[0]]) * 1000
                r_ = np.linalg.norm(rtip[nm] - rtip["thumb"]) * 1000
                mark = "  <- pinch" if h < 30 else ""
                L.append(f"    thumb-{nm:<8}                {h:7.1f}   {r_:7.1f}{mark}")

            L += ["", "  joint angles [deg]  (# = near a limit)"]
            for f_i, fname in enumerate(["index ", "middle", "ring  ", "thumb "]):
                row = []
                for j in range(4):
                    name = f"joint_{f_i * 4 + j}_0"
                    i = joints.index(name)
                    deg = np.degrees(q[i])
                    lo, hi = np.degrees(limits[i])
                    near = "#" if (deg - lo < 3 or hi - deg < 3) else " "
                    row.append(f"{deg:+7.1f}{near}")
                L.append(f"    {fname:4} " + "".join(row))

            L += ["", "  + / -  adjust scaling      r  reset stats      q  quit"]
            sys.stdout.write("\033[H\033[J" + "\n".join(L) + "\n")
            sys.stdout.flush()

    print("\nstopped")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nstopped")
