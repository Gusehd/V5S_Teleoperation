"""Glove bridge (ZMQ) receiver.

Receives the frames published by `bridge_cpp/manus_bridge.cpp` and converts them
to MANO 21 points. Nothing below this module depends on the MANUS SDK -- the
bridge is the boundary.

Wire format (little-endian):

    uint32  magic      = 'MANU'
    uint32  version    = 1
    uint64  t_ns                 send time (sender's monotonic clock)
    uint32  glove_id
    uint32  side                 1=left, 2=right
    uint32  node_count
    float32 nodes[node_count][7] (px,py,pz, qx,qy,qz,qw)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Self

import numpy as np
import zmq

MAGIC = 0x554E414D
VERSION = 1
_HEADER = struct.Struct("<IIQIII")
_FLOATS_PER_NODE = 7


class Side(IntEnum):
    INVALID = 0
    LEFT = 1
    RIGHT = 2

    @property
    def is_right(self) -> bool:
        return self is Side.RIGHT


@dataclass(frozen=True)
class GloveFrame:
    """One glove frame."""

    t_ns: int
    """Send time, on the sender's monotonic clock, so **never compare it directly
    with the receiver's clock**. Use it to measure inter-frame gaps or to check
    that the stream is progressing."""
    glove_id: int
    side: Side
    positions: np.ndarray
    """(node_count, 3) node positions in metres."""
    rotations: np.ndarray
    """(node_count, 4) node rotation quaternions (x, y, z, w)."""

    @property
    def node_count(self) -> int:
        return int(self.positions.shape[0])

    def to_mano21(self) -> np.ndarray:
        """Convert to MANO 21 points (21, 3), already wrist-centred and
        MANO-axis-aligned."""
        from v5s_teleop.hand.mano import raw_nodes_to_mano21, to_mano_frame

        return to_mano_frame(raw_nodes_to_mano21(self.positions), is_right=self.side.is_right)


def parse_frame(payload: bytes) -> GloveFrame:
    """Wire bytes to a `GloveFrame`.

    Raises:
        ValueError: on a magic, version or length mismatch. Failing loudly beats
            silently misreading a format that has changed.
    """
    if len(payload) < _HEADER.size:
        raise ValueError(f"frame too short: {len(payload)} bytes")

    magic, version, t_ns, glove_id, side, node_count = _HEADER.unpack_from(payload)
    if magic != MAGIC:
        raise ValueError(f"magic mismatch: 0x{magic:08X} (expected 0x{MAGIC:08X})")
    if version != VERSION:
        raise ValueError(
            f"bridge protocol version {version}, this receiver expects {VERSION}. "
            f"Check that bridge_cpp/manus_bridge.cpp has been rebuilt."
        )

    expected = _HEADER.size + node_count * _FLOATS_PER_NODE * 4
    if len(payload) != expected:
        raise ValueError(
            f"length mismatch: {len(payload)} bytes ({expected} for {node_count} nodes)")

    data = np.frombuffer(payload, dtype="<f4", offset=_HEADER.size).reshape(node_count, _FLOATS_PER_NODE)
    return GloveFrame(
        t_ns=t_ns,
        glove_id=glove_id,
        side=Side(side) if side in tuple(Side) else Side.INVALID,
        positions=data[:, 0:3].astype(np.float64),
        rotations=data[:, 3:7].astype(np.float64),
    )


class GloveSubscriber:
    """Bridge subscriber.

    With `CONFLATE=1` the queue only ever holds the newest frame, so a slow
    retargeting step does not accumulate a backlog of stale frames.

    Usage:

        with GloveSubscriber() as sub:
            while True:
                frame = sub.recv(timeout_ms=100)
                if frame is None:
                    continue          # the watchdog handles this
                mano21 = frame.to_mano21()
    """

    def __init__(self, endpoint: str = "tcp://127.0.0.1:5555"):
        self.endpoint = endpoint
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.setsockopt(zmq.LINGER, 0)
        # CONFLATE only works with a single subscription filter, so subscribe
        # to everything exactly once.
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._sock.connect(endpoint)

    def recv(self, timeout_ms: int = 100) -> GloveFrame | None:
        """Receive one frame, or None if none arrives in time."""
        if not self._sock.poll(timeout_ms):
            return None
        return parse_frame(self._sock.recv())

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *exc) -> None:
        self.close()
