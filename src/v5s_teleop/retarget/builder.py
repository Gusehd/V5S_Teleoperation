"""Config file to assembled retargeter.

We assemble the pieces here rather than using the upstream
`dex_retargeting.retargeting_config.RetargetingConfig`. There is exactly one
reason: **the upstream config class does not pass `eta1`/`eta2` to the
optimizer.** `DexPilotOptimizer.__init__` accepts them, but
`RetargetingConfig.build()` only forwards `project_dist`/`escape_dist`, so a
config file cannot reach `eta1` and it stays pinned at the default `1e-4`.

`eta1` is **the target distance between fingertip frames once a pinch counts as
closed**. On the V5 Sense the `link_*_tip` frame origins sit **inside** the
fingertip mesh (9.5-9.8 mm from the surface), so when two fingertip pads
physically touch, the frames are still about 27 mm apart. Leaving the upstream
default of `1e-4` makes the optimizer keep pushing to overlap the fingertips by
27 mm -- on real hardware this shows up as "the pinch never closes".

Every class assembled here (`RobotWrapper`, `DexPilotOptimizer`,
`SeqRetargeting`, `LPFilter`) is upstream public API. Upstream sources are
never modified.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from dex_retargeting.optimizer_utils import LPFilter
from dex_retargeting.robot_wrapper import RobotWrapper
from dex_retargeting.seq_retarget import SeqRetargeting

from v5s_teleop.retarget.optimizer_ext import SelfCollisionDexPilotOptimizer


@dataclass
class RetargetingSpec:
    """Schema of `configs/*.yml`.

    Fields whose names match upstream keep upstream's meaning, so upstream
    documentation reads correctly here. Our own additions are grouped under
    "extensions" below.
    """

    urdf_path: str
    wrist_link_name: str
    finger_tip_link_names: list[str]
    """The thumb goes **first**. DexPilot treats the first fingertip as the
    thumb when it builds the pinch pairs."""
    target_joint_names: list[str]
    scaling_factor: float = 1.0
    low_pass_alpha: float = 0.2
    project_dist: float = 0.03
    escape_dist: float = 0.05
    normal_delta: float = 4e-3
    """Penalty for moving away from the previous solution. Larger is stickier,
    smaller is more agile."""
    huber_delta: float = 2e-2
    """Beyond this error the loss becomes linear. Keeps a glove glitch from
    blowing up the solution."""

    # ── Extensions: things upstream does not expose through its config ────
    eta1: float = 1e-4
    """Target distance [m] for a closed thumb-to-finger pinch. **Always
    override this with a measured value** matching how far the fingertip frame
    sits inside the pad surface."""
    eta2: float = 3e-2
    """Target separation [m] between fingers. Keeps neighbouring fingers from
    intruding during a pinch."""
    collision_link_pairs: list[list] = field(default_factory=list)
    """List of `[link_a, link_b, margin_m]`. Keep the margin below the natural
    separation of an open hand."""
    collision_weight: float = 0.0

    shape_segments: list[list] = field(default_factory=list)
    """Segments used for shape matching:
    `[robot_start_link, robot_end_link, human_start_point, human_end_point]`.

    Human points are MANO 21 indices (index MCP=5, PIP=6, DIP=7, TIP=8, ...).
    What is matched is **which way each segment points** (its unit vector)."""

    shape_weight: float = 0.0
    """Shape matching strength. 0 turns the whole term off (the default).

    DexPilot constrains only fingertip positions, which leaves one degree of
    freedom undetermined across a finger's four joints -- more than eight joint
    combinations produce the same fingertip position. This term fills that
    freedom with the human hand's shape."""
    wrist_offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    """Places a **virtual wrist frame** this far [m] from `wrist_link_name`.

    DexPilot matches "wrist to fingertip" vectors between human and robot. But
    this hand's `palm_link` origin is not at the wrist -- it is at the **finger
    bases (the knuckles)**. A human wrist sits roughly 100 mm behind the
    knuckles, so without this the two reference points are misaligned while the
    vectors are compared. This is an **offset, not a scale**, so no amount of
    `scaling_factor` tuning removes it.

    An empty value ([0,0,0]) does nothing."""

    joint_limit_overrides: dict[str, list[float]] = field(default_factory=dict)
    """Per-joint `[lower, upper]` [rad] overrides, for when the URDF limits are
    far wider than a human hand. The V5 Sense already has a narrow abduction
    range (30-55 degrees), so this is usually left empty."""

    @classmethod
    def from_yaml(cls, path: str | Path) -> RetargetingSpec:
        path = Path(path)
        raw = yaml.safe_load(path.read_text())
        cfg = raw.get("retargeting", raw)

        declared = {f for f in cls.__dataclass_fields__}
        unknown = set(cfg) - declared - {"type"}
        if unknown:
            raise ValueError(
                f"{path.name}: unknown config key(s) {sorted(unknown)}. "
                f"A silently ignored typo would only surface on real hardware."
            )
        if "type" in cfg and str(cfg["type"]).lower() != "dexpilot":
            raise ValueError(
                f"{path.name}: only DexPilot is supported for type (got {cfg['type']})")

        # Relative URDF paths resolve against the config file's location.
        cfg = {k: v for k, v in cfg.items() if k != "type"}
        spec = cls(**cfg)
        if not Path(spec.urdf_path).is_absolute():
            spec.urdf_path = str((path.parent / spec.urdf_path).resolve())
        return spec


def build_retargeting(spec: RetargetingSpec | str | Path) -> SeqRetargeting:
    """Config to a ready-to-use `SeqRetargeting`."""
    if not isinstance(spec, RetargetingSpec):
        spec = RetargetingSpec.from_yaml(spec)

    if spec.eta1 <= 1e-3:
        # Refuse to fail silently. A config that inherited the upstream default
        # is almost always a mistake.
        raise ValueError(
            f"eta1={spec.eta1} is only valid when the fingertip frame sits at the "
            f"contact surface. If it sits inside the mesh, use a measured value that "
            f"accounts for that depth (V5 Sense measurement is about 0.027)."
        )

    urdf_path, wrist_link_name = spec.urdf_path, spec.wrist_link_name
    if any(abs(v) > 1e-9 for v in spec.wrist_offset):
        urdf_path, wrist_link_name = _urdf_with_virtual_wrist(
            spec.urdf_path, spec.wrist_link_name, spec.wrist_offset)

    robot = RobotWrapper(urdf_path)

    optimizer = SelfCollisionDexPilotOptimizer(
        robot,
        target_joint_names=spec.target_joint_names,
        finger_tip_link_names=spec.finger_tip_link_names,
        wrist_link_name=wrist_link_name,
        scaling=spec.scaling_factor,
        project_dist=spec.project_dist,
        escape_dist=spec.escape_dist,
        norm_delta=spec.normal_delta,
        huber_delta=spec.huber_delta,
        eta1=spec.eta1,
        eta2=spec.eta2,
        collision_link_pairs=[(a, b, m) for a, b, m in spec.collision_link_pairs],
        collision_weight=spec.collision_weight,
        shape_segments=[(a, b, int(ha), int(hb)) for a, b, ha, hb in spec.shape_segments],
        shape_weight=spec.shape_weight,
    )

    retargeting = SeqRetargeting(optimizer, has_joint_limits=True, lp_filter=LPFilter(spec.low_pass_alpha))

    # Joint limit overrides are applied **after** SeqRetargeting installs the URDF
    # limits. Upstream's SeqRetargeting.__init__ calls set_joint_limit, so doing it
    # in the other order would have our values overwritten.
    if spec.joint_limit_overrides:
        _apply_joint_limit_overrides(retargeting, spec.joint_limit_overrides)

    return retargeting


def _apply_joint_limit_overrides(
    retargeting: SeqRetargeting, overrides: dict[str, list[float]]
) -> None:
    """Force a range narrower than the URDF limits. Widening is rejected."""
    optimizer = retargeting.optimizer
    limits = np.array(retargeting.joint_limits, dtype=np.float64)  # (opt_dof, 2)
    names = list(optimizer.target_joint_names)

    for joint_name, bounds in overrides.items():
        if joint_name not in names:
            raise ValueError(
                f"joint_limit_overrides: '{joint_name}' is not a target joint")
        lower, upper = float(bounds[0]), float(bounds[1])
        if lower >= upper:
            raise ValueError(
                f"joint_limit_overrides['{joint_name}']: lower >= upper")
        i = names.index(joint_name)
        if lower < limits[i, 0] or upper > limits[i, 1]:
            raise ValueError(
                f"joint_limit_overrides['{joint_name}'] = [{lower}, {upper}] falls "
                f"outside the URDF limits [{limits[i, 0]:.3f}, {limits[i, 1]:.3f}]. "
                f"This setting narrows a range; it cannot widen one."
            )
        limits[i] = (lower, upper)

    optimizer.set_joint_limit(limits)
    retargeting.joint_limits = limits
    # If the warm-start point is outside the new limits, the first solution
    # would start out of bounds.
    retargeting.last_qpos = np.clip(retargeting.last_qpos, limits[:, 0], limits[:, 1]).astype(np.float32)


_VIRTUAL_WRIST_LINK = "virtual_wrist"


def _urdf_with_virtual_wrist(urdf_path: str, parent_link: str, offset: list[float]) -> tuple[str, str]:
    """Add a virtual wrist link to a copy of the URDF and return its path.

    The original URDF is left untouched. Kinematics are unaffected -- the link is
    fixed and has neither mass nor geometry, so it only serves as a reference
    point.

    Mesh paths may be relative, so the copy is written to **the same directory
    as the original**.
    """
    import xml.etree.ElementTree as ET

    src = Path(urdf_path)
    tree = ET.parse(src)
    root = tree.getroot()

    existing = {l.get("name") for l in root.findall("link")}
    if parent_link not in existing:
        raise ValueError(f"URDF has no link named '{parent_link}'")
    if _VIRTUAL_WRIST_LINK in existing:
        return str(src), _VIRTUAL_WRIST_LINK

    ET.SubElement(root, "link", {"name": _VIRTUAL_WRIST_LINK})
    joint = ET.SubElement(
        root, "joint", {"name": f"{_VIRTUAL_WRIST_LINK}_joint", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent_link})
    ET.SubElement(joint, "child", {"link": _VIRTUAL_WRIST_LINK})
    ET.SubElement(joint, "origin", {
        "xyz": " ".join(f"{v:.6f}" for v in offset), "rpy": "0 0 0"})

    out = src.with_name(f".{src.stem}.virtualwrist.urdf")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return str(out), _VIRTUAL_WRIST_LINK
