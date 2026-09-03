"""Large display of tactile pressure to glove vibration, for screen recording.

It uses **the same config and the same mapper** as `haptics_node`, so the
vibration values shown here are exactly what goes to the glove. This tool only
reads and never sends vibration, so it is safe to run alongside the haptics
node.

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    python tools/haptics_viz.py --num 0
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from v5s_teleop.haptics_map import FINGER_CHANNELS, TACTILE_LEN, HapticMapper
from v5s_teleop.ros2.haptics_node import _load_mapping

NAMES = ("THUMB", "INDEX", "MIDDLE", "RING")
W = 44                      # bar length


def _bar(frac: float, width: int, ch: str = "█") -> str:
    n = max(0, min(width, round(frac * width)))
    return ch * n + "·" * (width - n)


def _hue(frac: float) -> str:
    """Blue to green to yellow to red as contact strength rises."""
    if frac <= 0.01:
        return "\033[38;5;238m"
    if frac < 0.25:
        return "\033[38;5;39m"
    if frac < 0.55:
        return "\033[38;5;46m"
    if frac < 0.80:
        return "\033[38;5;226m"
    return "\033[38;5;196m"


class Viz(Node):
    def __init__(self, args):
        super().__init__("v5s_haptics_viz")
        self.mapping = _load_mapping(args.config, self.get_logger())
        self.mapper = HapticMapper(self.mapping)
        self.tactile: list[float] | None = None
        self.frames = 0
        self.hz = 0
        self.create_subscription(
            Float32MultiArray, f"allegroHand_{args.num}/tactile_sensors",
            self._on_tactile, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1 / 15.0, self._draw)
        self.create_timer(1.0, self._hz)
        sys.stdout.write("\033[2J\033[?25l")     # clear the screen, hide the cursor

    def _on_tactile(self, msg) -> None:
        if len(msg.data) == TACTILE_LEN:
            self.tactile = list(msg.data)
            self.frames += 1

    def _hz(self) -> None:
        self.hz, self.frames = self.frames, 0

    def _draw(self) -> None:
        m = self.mapping
        out = ["\033[H"]
        out.append("\033[1m  V5 SENSE  —  TACTILE → GLOVE HAPTICS\033[0m\n")
        out.append(f"\033[38;5;244m  {m.mode}   {m.min_kpa:.0f}~{m.max_kpa:.0f} kPa   "
                   f"gamma {m.gamma:.1f}   {self.hz} Hz\033[0m\n\n")
        if self.tactile is None:
            out.append("\033[38;5;208m  waiting for tactile data -- the hand driver must be running\033[0m\n")
            sys.stdout.write("".join(out)); sys.stdout.flush(); return

        powers = self.mapper(self.tactile)   # the same call haptics_node makes
        for i, name in enumerate(NAMES):
            kpa = self.tactile[FINGER_CHANNELS[i][1]]   # the tip of (segments, tip)
            pf = max(0.0, min(1.0, (kpa - m.min_kpa) / max(1e-6, m.max_kpa - m.min_kpa)))
            vib = powers[i]
            out.append(f"  \033[1m{name:<7}\033[0m\n")
            out.append(f"    pressure  {kpa:6.1f} kPa  {_hue(pf)}{_bar(pf, W)}\033[0m\n")
            out.append(f"    vibration {vib:6.2f}      {_hue(vib)}{_bar(vib, W, '▬')}\033[0m\n\n")
        sys.stdout.write("".join(out)); sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--num", default="0", help="the NUM in allegroHand_<NUM> (left = 0)")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    rclpy.init()
    node = Viz(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass   # Ctrl+C / SIGTERM -- avoid a messy traceback mid-recording
    finally:
        sys.stdout.write("\033[?25h\n")          # restore the cursor
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
