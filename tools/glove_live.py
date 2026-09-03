"""Live view of the glove stream.

Run it while the bridge (`bridge_cpp/manus_bridge`) is up. It shows fingertip
coordinates and the accumulated range of motion, updating live.

    .venv/bin/python tools/glove_live.py
"""

import sys
import time

import numpy as np

from v5s_teleop.bridge.zmq_client import GloveSubscriber

NAMES = ["wrist", "thumb tip", "index tip", "middle tip", "ring tip", "little tip"]
IDX = [0, 4, 8, 12, 16, 20]


def main() -> int:
    print("connecting to the bridge... (Ctrl+C to stop)\n")
    with GloveSubscriber() as sub:
        mn = mx = None
        n = 0
        t0 = time.time()
        last = 0.0
        while True:
            frame = sub.recv(timeout_ms=200)
            if frame is None:
                print("\rno frames from the bridge -- is manus_bridge running?", end="")
                continue
            m = frame.to_mano21()
            n += 1
            if mn is None:
                mn = m.copy()
                mx = m.copy()
            np.minimum(mn, m, out=mn)
            np.maximum(mx, m, out=mx)

            now = time.time()
            if now - last < 0.1:
                continue
            last = now
            span = np.linalg.norm(mx - mn, axis=1) * 1000

            lines = [
                (f"glove 0x{frame.glove_id:X}  {frame.side.name}  "
                 f"{frame.node_count} nodes  {n / (now - t0):.0f} Hz"),
                "",
                f"{'':8}{'current position (x, y, z) [mm]':^32}   range (min to max)",
            ]
            for k, i in enumerate(IDX):
                p = m[i] * 1000
                bar = "█" * min(40, int(span[i] / 4))
                lines.append(
                    f"{NAMES[k]:6}  ({p[0]:+7.1f},{p[1]:+7.1f},{p[2]:+7.1f})   "
                    f"{span[i]:6.1f} mm {bar}"
                )
            lines += [
                "",
                "The bar shows **how large a region that point has covered**, not path length.",
                "Fully closing and opening the hand reaches the maximum, after which it stops growing -- that is normal.",
                "Ctrl+C to stop.",
            ]
            sys.stdout.write("\033[H\033[J" + "\n".join(lines) + "\n")
            sys.stdout.flush()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nstopped")
