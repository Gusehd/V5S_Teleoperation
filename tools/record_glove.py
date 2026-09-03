"""Record glove postures for calibration.

Choosing the retargeting wrist reference offset and the human-to-robot scale
requires the operator's **actual range of motion**. The calibration file holds
only bone lengths -- it does not say how far the fingertip actually reaches
from the wrist when the hand is opened.

This tool walks through a few postures in order, collecting MANO 21-point
frames and saving them.

    .venv/bin/python tools/record_glove.py [output.npz]

The bridge (bridge_cpp/manus_bridge) must be running.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

from v5s_teleop.bridge.zmq_client import GloveSubscriber

# (label, instruction, seconds to record)
POSES: tuple[tuple[str, str, float], ...] = (
    ("open", "Open the hand wide, fingers **stretched out as far as they go**", 5.0),
    ("fist", "Hold a **tight fist**", 5.0),
    ("pinch_index", "Hold **thumb and index fingertips together**", 5.0),
    ("pinch_middle", "Hold **thumb and middle fingertips together**", 5.0),
    ("spread", "Keep the fingers straight and **spread them as wide as possible**", 5.0),
    ("free", "Move freely through **several postures, slowly** (grasp, open, pinch)", 12.0),
)


def countdown(msg: str, seconds: int = 3) -> None:
    for i in range(seconds, 0, -1):
        print(f"\r  {msg}  ...{i}", end="", flush=True)
        time.sleep(1)
    print("\r" + " " * 70, end="\r")


def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "data/glove_calib.npz")
    out.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 68)
    print(" glove calibration recording")
    print("=" * 68)
    print(" Follow each instruction and hold that posture while it records.")
    print(" The glove must be worn.\n")

    with GloveSubscriber() as sub:
        if sub.recv(timeout_ms=2000) is None:
            print("no frames from the bridge. Check that manus_bridge is running.")
            return 1

        recorded: dict[str, np.ndarray] = {}
        meta: dict[str, object] = {}

        for label, prompt, duration in POSES:
            print(f"\n[{label}]  {prompt}")
            countdown("get ready", 3)
            print(f"  * recording ({duration:.0f} s)", end="", flush=True)

            frames: list[np.ndarray] = []
            t0 = time.time()
            while time.time() - t0 < duration:
                f = sub.recv(timeout_ms=100)
                if f is None:
                    continue
                frames.append(f.to_mano21())
                meta.setdefault("glove_id", int(f.glove_id))
                meta.setdefault("side", f.side.name)

            if not frames:
                print("  -> 0 frames! The bridge may have stalled.")
                return 1
            recorded[label] = np.stack(frames)
            print(f"  -> {len(frames)} frames")

        np.savez_compressed(out, **recorded, **{f"meta_{k}": v for k, v in meta.items()})
        print(f"\nsaved: {out}")
        print(f"  glove 0x{meta['glove_id']:X} / {meta['side']}")
        for k, v in recorded.items():
            print(f"  {k:14} {v.shape[0]:5d} frames")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\naborted")
        sys.exit(130)
