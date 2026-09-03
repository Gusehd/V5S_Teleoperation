"""Tactile sensors to glove vibration. **A separate process that runs
independently of teleoperation.**

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    python -m v5s_teleop.ros2.haptics_node --num 0

Defaults come from `configs/haptics.yml`. Once good values are found, write them
back into that file.

The bridge must be started with haptic input for vibration to go out::

    ./bridge_cpp/manus_bridge --haptics tcp://127.0.0.1:5556

The bridge's haptic sockets are **per hand** (left 5556, right 5558). This node
connects to the address matching ``--hand`` on its own, so there is normally
nothing to specify. Since the socket identifies the hand, ``--glove-id`` is not
needed either -- the bridge sends to that hand's glove.

Without `--haptics` on the bridge, the entire bridge-side path is dead and
teleoperation is unaffected. This node can be stopped and started on its own
while teleoperation keeps running.

Every parameter is adjustable at runtime::

    ros2 param set /v5s_haptics mode step     # or linear
    ros2 param set /v5s_haptics min_kpa 5.0   # at or below this, no vibration
    ros2 param set /v5s_haptics max_kpa 40.0  # vibration is maximum here
    ros2 param set /v5s_haptics gamma 0.5     # smaller is more sensitive
"""

from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

import v5s_teleop
from v5s_teleop.bridge.haptics import HapticSender
from v5s_teleop.haptics_map import TACTILE_LEN, HapticMapper, HapticMapping

#: The bridge's per-hand haptic sockets, chosen not to collide with the
#: teleoperation streams (5555/5557).
HAPTIC_ENDPOINTS = {"left": "tcp://127.0.0.1:5556", "right": "tcp://127.0.0.1:5558"}

#: Only these are user-facing. The rest (motor floor, noise floor, hysteresis)
#: stay as internal constants in haptics_map.py, to keep the surface a user has
#: to touch after release as small as possible.
_TUNABLE = ("min_kpa", "max_kpa", "gamma")


class HapticsNode(Node):
    def __init__(self, args):
        # Per-hand node names for the same reason as the teleop node: running
        # both hands at once would otherwise collide.
        super().__init__(f"v5s_haptics_{args.hand}")
        self.mapping = _load_mapping(args.config, self.get_logger())
        self.mapper = HapticMapper(self.mapping)

        for name in _TUNABLE:
            self.declare_parameter(name, float(getattr(self.mapping, name)))
        self.declare_parameter("mode", self.mapping.mode)
        self._banner()
        self.declare_parameter("dry_run", bool(args.dry_run))
        self.add_on_set_parameters_callback(self._on_params)

        self.dry_run = args.dry_run
        self.sender = None if args.dry_run else HapticSender(args.haptic_endpoint,
                                                             glove_id=args.glove_id)
        if args.dry_run:
            self.get_logger().warn("DRY RUN -- not sending vibration, logging values only")
        else:
            self.get_logger().info(f"vibration output {args.haptic_endpoint}")

        topic = f"allegroHand_{args.num}/tactile_sensors"
        self.create_subscription(
            Float32MultiArray, topic, self._on_tactile,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(f"tactile subscription {topic}")

        self.frames = 0
        self.bad = 0
        self.last_powers = [0.0] * 5
        self.last_tactile: list[float] | None = None
        self.create_timer(1.0, self._report)

    def _banner(self) -> None:
        """Print the current settings prominently, and again whenever they change."""
        m = self.mapping
        self.get_logger().info(
            "─" * 58 + "\n"
            f"  mode      {m.mode:<8}  (linear = continuous / step = banded)\n"
            f"  min_kpa   {m.min_kpa:<8.1f}  at or below this, no vibration\n"
            f"  max_kpa   {m.max_kpa:<8.1f}  vibration is maximum here (1.0)\n"
            f"  gamma     {m.gamma:<8.2f}  smaller is more sensitive "
            f"(0.5 sensitive / 1 linear / 2 dull)\n"
            + "─" * 58 + "\n"
            "  change with:  ros2 param set /v5s_haptics <name> <value>")

    def _on_params(self, params) -> SetParametersResult:
        changed = False
        for p in params:
            if p.name in _TUNABLE:
                v = float(p.value)
                if p.name == "min_kpa" and v < 0:
                    return SetParametersResult(successful=False, reason="min_kpa must be at least 0")
                if p.name in ("max_kpa", "gamma") and v <= 0:
                    return SetParametersResult(successful=False, reason=f"{p.name} must be positive")
                setattr(self.mapping, p.name, v)
            elif p.name == "mode":
                if p.value not in ("linear", "step"):
                    return SetParametersResult(successful=False,
                                               reason="mode must be 'linear' or 'step'")
                self.mapping.mode = p.value
            else:
                continue
            changed = True
        if changed:
            self._banner()
        self.mapper.reset()
        return SetParametersResult(successful=True)

    def _on_tactile(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != TACTILE_LEN:
            self.bad += 1
            return
        self.last_tactile = list(msg.data)
        self.last_powers = self.mapper(self.last_tactile)
        self.frames += 1
        if self.sender is not None:
            self.sender.send(self.last_powers)

    def _report(self) -> None:
        if self.frames == 0:
            self.get_logger().warn(
                "no tactile data -- is the hand driver running?"
                + (f" ({self.bad} with a bad length)" if self.bad else ""))
        else:
            p = self.last_powers
            t = self.last_tactile or [0.0] * TACTILE_LEN
            tips = [t[3], t[7], t[11], t[15]]
            self.get_logger().info(
                f"{self.frames}Hz [{self.mapping.mode} "
                f"{self.mapping.min_kpa:.0f}~{self.mapping.max_kpa:.0f}kPa "
                f"g={self.mapping.gamma:.1f}] | tip kPa "
                + " ".join(f"{v:5.1f}" for v in tips)
                + " | vibration "
                + " ".join(f"{v:4.2f}" for v in p[:4]))
        self.frames = 0
        self.bad = 0

    def destroy_node(self) -> None:
        # Always turn vibration off on the way out, or the last strength stays
        # latched in the glove.
        if self.sender is not None:
            self.sender.close()
        super().destroy_node()


def _load_mapping(config: str | None, logger) -> HapticMapping:
    """Read the config file into a `HapticMapping`, falling back to the code
    defaults if it is missing."""
    import yaml

    path = Path(config) if config else (
        Path(v5s_teleop.__file__).parent / "configs/haptics.yml")
    if not path.exists():
        logger.warning(f"{path} not found, using code defaults")
        return HapticMapping()

    raw = yaml.safe_load(path.read_text()) or {}
    cfg = raw.get("haptics", raw)
    known = set(HapticMapping.__dataclass_fields__)
    unknown = set(cfg) - known
    if unknown:
        # Silently ignoring a typo leads to "I changed the value, why is nothing
        # different".
        raise ValueError(f"{path.name}: unknown config key(s) {sorted(unknown)}")
    logger.info(f"config {path.name}")
    return HapticMapping(**cfg)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None, help="haptics config yml (default configs/haptics.yml)")
    ap.add_argument("--hand", default="left", choices=("left", "right"),
                    help="appended to the node name (v5s_haptics_<HAND>); needed when running both hands")
    ap.add_argument("--num", default="0", help="the NUM in allegroHand_<NUM> (left = 0)")
    ap.add_argument("--haptic-endpoint", default=None,
                    help="defaults to the socket matching --hand "
                         f"({HAPTIC_ENDPOINTS['left']} / {HAPTIC_ENDPOINTS['right']})")
    ap.add_argument("--glove-id", type=lambda v: int(v, 0), default=0,
                    help="0 lets the bridge use the glove of the hand that socket "
                         "serves; normally there is no need to set this")
    ap.add_argument("--dry-run", action="store_true",
                    help="do not send vibration, only log the values")
    args = ap.parse_args()
    if args.haptic_endpoint is None:
        args.haptic_endpoint = HAPTIC_ENDPOINTS[args.hand]

    rclpy.init()
    node = HapticsNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
