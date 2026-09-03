"""Glove vibration sender -- pushes to the bridge's haptic input socket.

MANUS Core Integrated allows only one instance system-wide to hold the SDK.
Measured (2026-08-21): while the bridge is running, any other process that tries
to connect gets a `Make sure to shut down all other instances of Core Integrated`
warning, receives no data, and takes the existing bridge stream down with it.

Vibration commands therefore go through the bridge, which owns the SDK. This
module is a thin sender to that socket and has no direct MANUS SDK dependency.

Wire format (little-endian):

    uint32  magic = 'MHAP'
    uint32  glove_id     0 lets the bridge use the hand that socket serves
    float32 powers[5]    Thumb, Index, Middle, Ring, Pinky -- each 0.0 to 1.0
"""

from __future__ import annotations

import struct
from collections.abc import Sequence
from typing import Self

import zmq

MAGIC = 0x5041484D
_PACKER = struct.Struct("<II5f")

#: Vibration channel order fixed by the SDK. There is exactly **one** channel
#: per finger (`CoreSdk_VibrateFingersForGlove` takes a single float[5]).
FINGER_ORDER: tuple[str, ...] = ("thumb", "index", "middle", "ring", "pinky")


class HapticSender:
    """Sends vibration commands to the bridge's haptic socket.

    The bridge must be started with `--haptics <endpoint>` for anything to be
    listening. If it is not, this side still works without error (a ZMQ PUSH
    socket queues and then discards when there is no peer).

        with HapticSender() as h:
            h.send([0.0, 0.6, 0.0, 0.0, 0.0])   # index finger only
            h.stop()
    """

    def __init__(self, endpoint: str = "tcp://127.0.0.1:5556", glove_id: int = 0):
        # The default is the left-hand socket. The right hand is 5558 --
        # HAPTIC_ENDPOINTS in ros2/haptics_node.py picks it from --hand.
        self.endpoint = endpoint
        self.glove_id = int(glove_id)
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUSH)
        # CONFLATE keeps only the newest command. There is no reason to replay a
        # backlog of stale vibration values.
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(endpoint)

    def send(self, powers: Sequence[float]) -> None:
        """Send five vibration strengths (Thumb, Index, Middle, Ring, Pinky).

        Raises:
            ValueError: if there are not exactly five values.
        """
        if len(powers) != 5:
            raise ValueError(f"powers must have 5 entries, got {len(powers)}")
        clamped = [min(1.0, max(0.0, float(v))) for v in powers]
        self._sock.send(_PACKER.pack(MAGIC, self.glove_id, *clamped), zmq.DONTWAIT)

    def stop(self) -> None:
        """Turn every channel off. Always call before exiting, or the last
        vibration strength stays latched in the glove."""
        self.send([0.0] * 5)

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self._sock.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *exc) -> None:
        self.close()
