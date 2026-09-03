"""MANUS glove skeleton to the MANO 21-keypoint convention.

Ported from our own `manus-isaacsim-teleop/scripts/v6_retarget.py`. The
retargeting core (dex-retargeting) expects a 21-point array expressed in a
wrist-origin, MANO-aligned frame, so this is where the glove's node stream is
converted to that convention.
"""

from __future__ import annotations

import numpy as np

# MANO/MediaPipe 21-keypoint order, listed by glove node name. Indices 0 (wrist),
# 5 (index MCP) and 9 (middle MCP) are the ones used to estimate the wrist frame.
MANO21_NODE_ORDER: tuple[str, ...] = (
    "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_proximal", "index_intermediate", "index_distal", "index_tip",
    "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
    "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
    "little_proximal", "little_intermediate", "little_distal", "little_tip",
)

# MANUS raw skeleton (25 nodes) to MANO 21-point index map.
#
# Node layout reported by the glove (measured with tools/manus_nodes, 2026-08-20):
#     0        hand (wrist)
#     1-4      thumb  : metacarpal, proximal, distal, tip
#     5-9      index  : metacarpal, proximal, intermediate, distal, tip
#     10-14    middle : ditto
#     15-19    ring   : ditto
#     20-24    little : ditto
#
# MANO's 21 points do not use the finger metacarpals (the thumb metacarpal is
# used). So only nodes 5, 10, 15 and 20 are dropped; the other 21 map directly.
MANUS_RAW_TO_MANO21: tuple[int, ...] = (
    0,                    # wrist            <- hand
    1, 2, 3, 4,           # thumb            <- metacarpal, proximal, distal, tip
    6, 7, 8, 9,           # index            <- from proximal on (metacarpal 5 dropped)
    11, 12, 13, 14,       # middle           <- (metacarpal 10 dropped)
    16, 17, 18, 19,       # ring             <- (metacarpal 15 dropped)
    21, 22, 23, 24,       # little           <- (metacarpal 20 dropped)
)

MANUS_RAW_NODE_COUNT = 25

# Operator frame to MANO frame rotation. The two differ by a 180-degree turn
# about x, not by a reflection -- so the right-hand path must not be reused for
# the left by simply mirroring the keypoints.
OPERATOR2MANO_RIGHT = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=np.float64)
OPERATOR2MANO_LEFT = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=np.float64)


def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
    """Estimate the wrist orientation frame (MANO convention) from 21 keypoints.

    Takes the plane through the wrist (0), index MCP (5) and middle MCP (9),
    finds its normal by SVD, and picks an x axis inside that plane. This is more
    robust to hand posture changes than a fixed canonical rotation.

    Args:
        keypoint_3d_array: (21, 3) keypoints relative to the wrist origin.

    Returns:
        (3, 3) rotation matrix whose columns are the x, normal and z axes.
    """
    if keypoint_3d_array.shape != (21, 3):
        raise ValueError(f"expected (21, 3), got {keypoint_3d_array.shape}")
    points = keypoint_3d_array[[0, 5, 9], :]

    x_vector = points[0] - points[2]

    points = points - np.mean(points, axis=0, keepdims=True)
    _, _, v = np.linalg.svd(points)
    normal = v[2, :]

    # Project x onto the plane so it is orthogonal to the normal.
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / np.linalg.norm(x)
    z = np.cross(x, normal)

    # SVD does not fix the sign of the normal, so anchor it consistently
    # towards the index MCP.
    if np.sum(z * (points[1] - points[2])) < 0:
        normal *= -1
        z *= -1
    return np.stack([x, normal, z], axis=1)


def raw_nodes_to_mano21(node_positions: np.ndarray) -> np.ndarray:
    """MANUS raw skeleton (25, 3) to MANO 21 points (21, 3).

    Takes the node array straight from the glove bridge. Selection is by index
    rather than by name lookup, so there is no per-frame cost.

    Raises:
        ValueError: if the node count is not 25. A different glove model may use
            a different layout, so this must not pass silently.
    """
    arr = np.asarray(node_positions, dtype=np.float64)
    if arr.shape != (MANUS_RAW_NODE_COUNT, 3):
        raise ValueError(
            f"MANUS raw skeleton must be ({MANUS_RAW_NODE_COUNT}, 3), "
            f"got {arr.shape}. Re-check the node layout with tools/manus_nodes."
        )
    return arr[list(MANUS_RAW_TO_MANO21), :]


def nodes_to_mano21(node_positions: dict[str, np.ndarray]) -> np.ndarray:
    """Glove node dict to a (21, 3) array in MANO order.

    Raises:
        KeyError: if any node in `MANO21_NODE_ORDER` is missing.
    """
    return np.stack(
        [np.asarray(node_positions[name][:3], dtype=np.float64) for name in MANO21_NODE_ORDER]
    )


def to_mano_frame(mano21: np.ndarray, is_right: bool) -> np.ndarray:
    """Move (21, 3) keypoints into the wrist-origin, MANO-aligned frame."""
    operator2mano = OPERATOR2MANO_RIGHT if is_right else OPERATOR2MANO_LEFT
    wrist_centered = mano21 - mano21[0:1, :]
    wrist_rot = estimate_frame_from_hand_points(wrist_centered)
    return wrist_centered @ wrist_rot @ operator2mano
