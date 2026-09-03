#!/usr/bin/env python3
"""See which physical glove arrives on which port.

Run it with only the bridge up and **shake one glove at a time**. The motion
bar on that glove's row grows. A swapped left/right mapping shows up here
immediately.

    ./bridge_cpp/manus_bridge                       # terminal 1
    .venv/bin/python tools/check_lr_mapping.py      # terminal 2

How to read it::

    left  5555  0x1EC9928C  side=left   113Hz  motion ████████        42.1 mm/s
    right 5557  0x4D0CA36C  side=right    0Hz  motion                  0.0 mm/s

`side` is the value the bridge put on the wire. If shaking the physical left
glove makes the **right row** react, the left/right mapping is inverted.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from v5s_teleop.bridge.zmq_client import GloveSubscriber

PORTS = {"left ": "tcp://127.0.0.1:5555", "right": "tcp://127.0.0.1:5557"}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=0.0, help="0 means run until Ctrl+C")
    args = ap.parse_args()

    subs = {name: GloveSubscriber(ep) for name, ep in PORTS.items()}
    prev: dict[str, np.ndarray] = {}
    speed = dict.fromkeys(PORTS, 0.0)      # mm/s, exponentially smoothed
    count = dict.fromkeys(PORTS, 0)
    info = {name: ("—", "—") for name in PORTS}

    print("shake one glove at a time.  Ctrl+C to stop\n")
    t_report = time.perf_counter()
    t_end = time.perf_counter() + args.seconds if args.seconds else None
    try:
        while t_end is None or time.perf_counter() < t_end:
            for name, sub in subs.items():
                f = sub.recv(timeout_ms=2)
                if f is None:
                    continue
                count[name] += 1
                info[name] = (f"0x{f.glove_id:08X}", f.side.name.lower())
                p = f.positions
                if name in prev and prev[name].shape == p.shape:
                    # Mean node displacement between frames; grows when shaken.
                    d = float(np.linalg.norm(p - prev[name], axis=1).mean()) * 1000.0
                    speed[name] = 0.8 * speed[name] + 0.2 * d * 100.0   # roughly mm/s
                prev[name] = p

            now = time.perf_counter()
            if now - t_report >= 0.5:
                print("\033[2A\033[J" if t_report else "", end="")
                for name in PORTS:
                    gid, side = info[name]
                    hz = int(count[name] / (now - t_report))
                    bars = "█" * min(20, int(speed[name] / 5))
                    print(f"{name} {PORTS[name][-4:]}  {gid}  side={side:<6}"
                          f"{hz:4d}Hz  motion {bars:<20} {speed[name]:6.1f} mm/s")
                    count[name] = 0
                t_report = now
    except KeyboardInterrupt:
        pass
    finally:
        for s in subs.values():
            s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
