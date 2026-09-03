"""Live view of the 16 tactile channels -- for learning the value range and noise.

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
    python tools/tactile_live.py --num 0

The hand driver must be running. No vibration is sent.
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from v5s_teleop.haptics_map import FINGER_CHANNELS, PALM_INDEX, TACTILE_LEN

NAMES = ("thumb ", "index ", "middle", "ring  ")


class TactileLive(Node):
    def __init__(self, num: str):
        super().__init__("tactile_live")
        self.lo = [float("inf")] * TACTILE_LEN
        self.hi = [float("-inf")] * TACTILE_LEN
        self.n = 0
        self.create_subscription(
            Float32MultiArray, f"allegroHand_{num}/tactile_sensors", self.cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def cb(self, msg: Float32MultiArray) -> None:
        d = list(msg.data)
        if len(d) != TACTILE_LEN:
            return
        self.n += 1
        for i, v in enumerate(d):
            self.lo[i] = min(self.lo[i], v)
            self.hi[i] = max(self.hi[i], v)
        if self.n % 10:
            return
        L = [f"{self.n} frames    (current / observed min-max)", ""]
        L.append(f"  palm    {d[PALM_INDEX]:8.2f}   [{self.lo[PALM_INDEX]:7.2f} ~ {self.hi[PALM_INDEX]:7.2f}]")
        L.append("")
        for name, (madi, tip) in zip(NAMES, FINGER_CHANNELS):
            bar = "█" * min(40, int(max(0.0, d[tip])))
            L.append(f"  {name} tip {d[tip]:8.2f}   [{self.lo[tip]:7.2f} ~ {self.hi[tip]:7.2f}]  {bar}")
            L.append("         seg " + "  ".join(f"{d[j]:7.2f}" for j in madi))
        L += ["", "Press the fingers to learn the range. Ctrl+C to stop."]
        sys.stdout.write("\033[H\033[J" + "\n".join(L) + "\n")
        sys.stdout.flush()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--num", default="0")
    args = ap.parse_args()
    rclpy.init()
    node = TactileLive(args.num)
    print("waiting for tactile data... (the hand driver must be running)")
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
