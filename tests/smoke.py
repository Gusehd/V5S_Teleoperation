#!/usr/bin/env python3
"""Smoke tests -- catch what breaks silently after a bulk edit.

    .venv/bin/python tests/smoke.py        # exit code 0 on success

**Changes no existing code.** It only imports and reads.
No `pytest` -- that keeps the dependency list short and lets whoever receives
this repository run it straight away.

What it prevents:

    1. broken config or URDF paths -- today that only shows up on hardware
    2. **thumb commands going to the middle finger joints** -- pinocchio's
       order is not alphabetical
    3. rclpy leaking into the core -- retargeting must be verifiable without ROS
    4. retargeting output leaving the URDF limits
    5. the wire format drifting away from the bridge
    6. tactile channels mapping to the wrong vibration channel

This has a different job from `tools/check_lr_symmetry.py`, which is a
left/right symmetry regression check and is slow because it runs the optimizer
hundreds of times. This one only looks for "completely broken and nobody
noticed".
"""
from __future__ import annotations

import ast
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CONFIGS = {h: ROOT / f"src/v5s_teleop/configs/v5s_{h}_dexpilot.yml" for h in ("left", "right")}

#: The joint order the driver expects; must match teleop_node.DRIVER_JOINT_ORDER.
#: It is repeated here because **teleop_node requires rclpy** and cannot be
#: imported without ROS. Test 2b catches the two drifting apart.
DRIVER_JOINT_ORDER = tuple(f"joint_{i}_0" for i in range(16))

#: The core modules, i.e. those that must not depend on ROS.
CORE_MODULES = (
    "v5s_teleop.hand.mano",
    "v5s_teleop.haptics_map",
    "v5s_teleop.bridge.zmq_client",
    "v5s_teleop.bridge.haptics",
    "v5s_teleop.retarget.builder",
    "v5s_teleop.retarget.optimizer_ext",
)

_CHECKS: list[tuple[str, object]] = []


def check(name: str):
    """Register a test. main runs them in registration order."""
    def deco(fn):
        _CHECKS.append((name, fn))
        return fn
    return deco


# ── 1. Do both hands' configs build? ─────────────────────────────────
_built: dict[str, object] = {}


@check("1   both configs build")
def _build():
    from v5s_teleop.retarget.builder import build_retargeting
    for hand, path in CONFIGS.items():
        if not path.exists():
            raise FileNotFoundError(path)
        _built[hand] = build_retargeting(str(path))
    left = _built["left"].optimizer
    return f"scaling={left.scaling} shape={left.shape_weight} eta1={left.eta1}"


# ── 2. Joint reordering ──────────────────────────────────────────────
# pinocchio returns joints in kinematic-tree order, not alphabetically:
#     solver  0 1 2 3 [12 13 14 15] 4 5 6 7 8 9 10 11
#     driver  0 1 2 3 [ 4  5  6  7] 8 9 10 11 12 13 14 15
# Without the reorder, **thumb commands (12-15) go to the middle finger.**
@check("2a  the reorder permutation is correct by name")
def _perm():
    for hand, ret in _built.items():
        solver = list(ret.optimizer.robot.dof_joint_names)
        missing = set(DRIVER_JOINT_ORDER) - set(solver)
        if missing:
            raise AssertionError(f"{hand}: joints not in the URDF: {sorted(missing)}")
        to_driver = [solver.index(j) for j in DRIVER_JOINT_ORDER]
        # Applying the permutation must reproduce the driver order exactly.
        got = [solver[i] for i in to_driver]
        if got != list(DRIVER_JOINT_ORDER):
            raise AssertionError(f"{hand}: reordered {got[:6]}... != driver order")
        if len(set(to_driver)) != 16:
            raise AssertionError(f"{hand}: duplicate entries in the permutation {to_driver}")
    return "thumb 12-15 land in the right slots"


@check("2b  the reorder is computed in exactly one place")
def _perm_dup():
    # The same computation used to be copied into __init__ and the parameter
    # rebuild path. Fixing only one would scramble the commands the moment
    # wrist_offset changed. They are merged into _bind_driver_order(); this
    # catches them splitting apart again.
    src = (ROOT / "src/v5s_teleop/ros2/teleop_node.py").read_text()
    tree = ast.parse(src)
    sites = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Attribute) and t.attr == "to_driver" for t in n.targets)
    ]
    if len(sites) != 1:
        raise AssertionError(
            f"to_driver is assigned in {len(sites)} places -- there must be one")
    if 'f"joint_{i}_0" for i in range(16)' not in src:
        raise AssertionError(
            "the DRIVER_JOINT_ORDER definition changed -- update the constant here too")
    return "_bind_driver_order(), one place"


# ── 3. Does the core import without ROS? ─────────────────────────────
@check("3   the core imports without rclpy")
def _no_ros():
    # Import every core module with rclpy blocked. Unlike a textual scan, this
    # catches transitive dependencies too.
    # WARNING: find_module was removed in Python 3.12. The hook only works with
    # find_spec.
    code = (
        "import sys\n"
        "class Block:\n"
        "    def find_spec(self, name, path=None, target=None):\n"
        "        if name == 'rclpy' or name.startswith('rclpy.'):\n"
        "            raise ImportError('rclpy: the core must not depend on ROS')\n"
        "        return None\n"
        "sys.meta_path.insert(0, Block())\n"
        f"for m in {CORE_MODULES!r}:\n"
        "    __import__(m)\n"
    )
    r = subprocess.run([sys.executable, "-c", code], cwd=ROOT, check=False,
                       capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        raise AssertionError(r.stderr.strip().splitlines()[-1])
    return f"{len(CORE_MODULES)} modules"


# ── 4. Is the retargeting output inside the limits? ──────────────────
@check("4a  output stays near the URDF limits")
def _limits():
    sys.path.insert(0, str(ROOT / "tools"))
    import check_lr_symmetry as sym

    from v5s_teleop.hand.mano import raw_nodes_to_mano21, to_mano_frame

    worst = 0.0
    for hand, ret in _built.items():
        opt = ret.optimizer
        # WARNING: there are two orderings. retarget() returns pinocchio order,
        #   while joint_limits is in target order (SeqRetargeting filters it
        #   through idx_pin2target). Mixing them compares unrelated joints and
        #   produces a fake failure of over 100 degrees.
        pin2tgt = opt.idx_pin2target
        lo, hi = ret.joint_limits[:, 0], ret.joint_limits[:, 1]
        o, t = opt.target_link_human_indices
        for curl in (0.0, 0.5, 1.0):
            raw = sym.synth_left_hand(curl, curl)
            if hand == "right":
                raw = raw * sym.MIRROR
            m = to_mano_frame(raw_nodes_to_mano21(raw), is_right=(hand == "right"))
            opt.set_human_keypoints(m)
            q = ret.retarget(m[t] - m[o])[pin2tgt]      # -> target order
            over = float(max(np.max(lo - q), np.max(q - hi), 0.0))
            worst = max(worst, over)
    # SLSQP can return a value about 1e-3 rad outside the bound. That is why the
    # hard clamp in 4b exists; here we only check it does not go *far* out.
    if worst > 0.02:                       # 1.15°
        raise AssertionError(
            f"exceeds the limits by {np.degrees(worst):.3f} deg -- beyond numerical slop")
    return f"6 poses, max excess {np.degrees(worst):.3f} deg (numerical slop)"


@check("4b  a hard clamp is applied before publishing")
def _clamp():
    # Hardware safety rule 1: do not trust the optimizer.
    src = (ROOT / "src/v5s_teleop/ros2/teleop_node.py").read_text()
    tree = ast.parse(src)
    clipped = {
        t.id
        for n in ast.walk(tree)
        if isinstance(n, ast.Assign)
        and isinstance(n.value, ast.Call)
        and ast.unparse(n.value.func).endswith("clip")
        and "to_driver" in ast.unparse(n.value.args[0])
        for t in n.targets
        if isinstance(t, ast.Name)
    }
    if not clipped:
        raise AssertionError("no np.clip is applied to qpos[to_driver]")
    pos = [
        ast.unparse(n.value)
        for n in ast.walk(tree)
        if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Attribute) and t.attr == "position" for t in n.targets)
    ]
    if not pos:
        raise AssertionError("could not find the msg.position assignment")
    if not any(any(c in p for c in clipped) for p in pos):
        raise AssertionError(f"msg.position does not use the clamped value: {pos}")
    return "msg.position <- np.clip(...)"


# ── 5. Wire format ───────────────────────────────────────────────────
@check("5   glove wire format round-trip")
def _wire():
    from v5s_teleop.bridge.zmq_client import MAGIC, VERSION, Side, parse_frame

    n = 25
    rng = np.random.default_rng(0)
    pos = rng.normal(size=(n, 3))
    rot = rng.normal(size=(n, 4))
    payload = struct.pack("<IIQIII", MAGIC, VERSION, 12345, 0x1EC9928C, int(Side.RIGHT), n)
    payload += np.concatenate([pos, rot], axis=1).astype(np.float32).tobytes()

    f = parse_frame(payload)
    if f.node_count != n or f.glove_id != 0x1EC9928C or f.side is not Side.RIGHT:
        raise AssertionError(f"header mismatch: {f.node_count} {f.glove_id:#x} {f.side}")
    if not np.allclose(f.positions, pos.astype(np.float32)):
        raise AssertionError("positions changed during the round-trip")
    # A corrupt payload must fail immediately rather than pass silently.
    for bad, why in ((payload[:-4], "a short frame"),
                     (b"\x00" * len(payload), "a magic mismatch")):
        try:
            parse_frame(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"{why} was accepted")
    return f"{n} nodes, corrupt frames rejected"


# ── 6. Tactile to vibration channels ─────────────────────────────────
@check("6   each tactile channel drives its own finger")
def _haptics():
    from v5s_teleop.haptics_map import FINGER_CHANNELS, TACTILE_LEN, HapticMapper, HapticMapping

    m = HapticMapper(HapticMapping())
    for i, (_madi, tip) in enumerate(FINGER_CHANNELS):
        t = [0.0] * TACTILE_LEN
        t[tip] = 1e3                      # a pressure that is certainly maximum
        p = m(t)
        m.reset()
        if len(p) != 5:
            raise AssertionError(f"expected 5 vibration channels, got {len(p)}")
        if p[i] <= 0.0:
            raise AssertionError(f"channel {tip} was pressed but finger {i} vibration is 0")
        other = [v for j, v in enumerate(p) if j != i]
        if any(v > 0.0 for v in other):
            raise AssertionError(f"channel {tip} also drove other fingers: {p}")
    return f"{len(FINGER_CHANNELS)} fingers, no crosstalk"


def main() -> int:
    print("smoke tests\n")
    failures = 0
    for name, fn in _CHECKS:
        try:
            detail = fn() or ""
        except Exception as e:  # noqa: BLE001 -- every failure is worth reporting
            failures += 1
            print(f"  FAIL  {name}\n          {type(e).__name__}: {e}")
        else:
            print(f"  ok    {name}" + (f"  ({detail})" if detail else ""))
    print()
    if failures:
        print(f"{failures} failed")
        return 1
    print(f"all passed ({len(_CHECKS)} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
