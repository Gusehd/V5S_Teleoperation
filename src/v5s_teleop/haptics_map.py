"""Tactile sensor readings to glove vibration strength.

The V5 Sense hand has 16 tactile channels, but the MANUS glove has **only one
vibration channel per finger, five in total** (`CoreSdk_VibrateFingersForGlove`
takes a float[5]). Each finger must therefore be reduced to a single value, and
we send **fingertip pressure 1:1**.

Why only the fingertip: with a single channel, mixing phalanx and fingertip
readings gives the wearer no way to tell where contact happened, and a vibration
felt at the fingertip that is driven by a phalanx sensor puts the sensation and
its cause in different places. (The phalanx indices are kept in
`FINGER_CHANNELS`, so they are available later if enveloping grasps need them.)

There are **only three** tunable parameters: `min_kpa`, `max_kpa` and `gamma`.
This is deliberate -- it keeps the surface a user has to touch after release
as small as possible.

This module depends on neither ROS nor the MANUS SDK. Being pure functions, it
is easy to test.
"""

from __future__ import annotations

from dataclasses import dataclass

# ─────────────────────────────────────────────────────────────────────────
# Layout of the 16 channels in allegroHand_<N>/tactile_sensors
# (std_msgs/Float32MultiArray). Confirmed in the driver's
# `allegro_node.cpp: publishData()`:
#   [0]=palm, [1-3]=thumb (phalanx, phalanx, tip), [4-7]=index (phalanx x3, tip),
#   [8-11]=middle (phalanx x3, tip), [12-15]=ring (phalanx x3, tip)
TACTILE_LEN = 16
PALM_INDEX = 0
#: Per finger: (phalanx indices, fingertip index). The order follows the SDK's
#: vibration channel order.
FINGER_CHANNELS: tuple[tuple[tuple[int, ...], int], ...] = (
    ((1, 2), 3),          # thumb
    ((4, 5, 6), 7),       # index
    ((8, 9, 10), 11),     # middle
    ((12, 13, 14), 15),   # ring
)
#: Our hand has four fingers. The glove's fifth channel (pinky) is always 0.
N_GLOVE_CHANNELS = 5

#: Band edges for `mode="step"` -- expressed as **fractions of `max_kpa`**.
#: They match the colour bands in the hand visualiser (10/20/30 at max 40 kPa).
#: The lowest band (blue, 0-5) is not listed because `min_kpa` already cuts it.
#: Being fractions, the edges follow automatically when `max_kpa` changes.
STEP_EDGES: tuple[float, ...] = (10 / 40, 20 / 40, 30 / 40)

#: Minimum strength at which the vibration motor actually starts turning.
#: Anything below this would be a "touching but feeling nothing" band, so we
#: raise the floor.
_MOTOR_FLOOR = 0.12

#: Hysteresis -- once on, stay on until the signal drops by this factor.
#: Stops the vibration flickering around the threshold.
_HYSTERESIS = 1.4


@dataclass
class HapticMapping:
    """Pressure to vibration conversion rule. **Two values to tune.**

    Two modes, switchable at runtime.

    **linear** -- strength varies continuously with pressure::

        t = (pressure - min_kpa) / (max_kpa - min_kpa)      (clamped to 0-1)
        vibration = t ** gamma

    **step** -- pressure is bucketed into bands with fixed strengths. People are
    poor at telling small differences in vibration strength apart, but good at
    noticing **the moment a band changes**. The edges match the colour bands in
    the hand visualiser, so what is on screen and what is felt line up. Each
    band's strength is its upper edge fed through the linear formula, so `gamma`
    means the same thing in both modes.
    """

    mode: str = "linear"
    """`"linear"` or `"step"`."""

    min_kpa: float = 5.0
    """Pressure at or below this counts as no vibration (dead zone).

    The sensors read slightly above zero even with no contact (operator
    measurement, 2026-08-24), so without this the glove would buzz constantly.
    The visualiser's blue band (0-5) covers the same region.

    Raise it and a firm press is needed; lower it and light contact registers,
    at the cost of picking up noise."""

    max_kpa: float = 40.0
    """Pressure at which vibration reaches maximum (1.0). Anything above is 1.0.
    The `min_kpa` to `max_kpa` range is spread across vibration 0 to 1.

    The sensor range is 0-400 kPa, but **the useful range is 0-40** (operator
    measurement, 2026-08-24 -- the visualiser turns red around 40). Unit is kPa.

    Lower it and light force reaches full vibration; raise it and a harder press
    is needed."""

    gamma: float = 1.0
    """How fast vibration grows with pressure: `vibration = normalized ** gamma`

    * `0.5` rises early -- even a graze is felt clearly (sensitive)
    * `1.0` proportional to pressure (linear)
    * `2.0` needs a firm press to respond (dull)
    """


class HapticMapper:
    """Applies a `HapticMapping` and keeps the hysteresis state."""

    def __init__(self, mapping: HapticMapping | None = None):
        self.mapping = mapping or HapticMapping()
        self._on = [False] * len(FINGER_CHANNELS)

    def reset(self) -> None:
        self._on = [False] * len(FINGER_CHANNELS)

    def _normalize(self, kpa: float) -> float:
        """Pressure [kPa] to 0-1: 0 at min_kpa, 1 at max_kpa."""
        m = self.mapping
        return min(1.0, max(0.0, (kpa - m.min_kpa) / (m.max_kpa - m.min_kpa)))

    def _curve(self, normalized: float) -> float:
        """Normalized pressure (0-1) to vibration strength (0-1), including the
        motor floor."""
        return _MOTOR_FLOOR + (1.0 - _MOTOR_FLOOR) * (normalized ** self.mapping.gamma)

    def _power(self, kpa: float) -> float:
        m = self.mapping
        if m.mode == "step":
            # Each band's strength is its **upper edge** through the linear
            # formula. The top band's upper edge is max_kpa itself (= 1.0).
            band = sum(1 for e in STEP_EDGES if kpa >= e * m.max_kpa)
            edge_kpa = (STEP_EDGES[band] * m.max_kpa
                        if band < len(STEP_EDGES) else m.max_kpa)
            return self._curve(self._normalize(edge_kpa))
        return self._curve(self._normalize(kpa))

    def __call__(self, tactile: list[float]) -> list[float]:
        """16 tactile channels to 5 vibration channels (Thumb, Index, Middle,
        Ring, Pinky).

        Raises:
            ValueError: if the input length is not 16, or a setting is invalid.
                Failing immediately beats silently driving the wrong channel
                after a driver change.
        """
        m = self.mapping
        if len(tactile) != TACTILE_LEN:
            raise ValueError(
                f"tactile array must have {TACTILE_LEN} entries, got {len(tactile)}. "
                f"Check the driver's tactile_sensors layout."
            )
        if m.mode not in ("linear", "step"):
            raise ValueError(f"mode must be 'linear' or 'step', got {m.mode!r}")
        if m.max_kpa <= m.min_kpa:
            raise ValueError(
                f"max_kpa ({m.max_kpa}) must be greater than min_kpa ({m.min_kpa})")
        if m.min_kpa < 0:
            raise ValueError(f"min_kpa must be at least 0, got {m.min_kpa}")
        if m.gamma <= 0:
            raise ValueError(f"gamma must be positive, got {m.gamma}")

        powers = [0.0] * N_GLOVE_CHANNELS
        for i, (_phalanx, tip) in enumerate(FINGER_CHANNELS):
            kpa = max(0.0, tactile[tip])

            # Hysteresis: the on threshold is min_kpa, the off threshold lower.
            threshold = m.min_kpa / _HYSTERESIS if self._on[i] else m.min_kpa
            if kpa <= threshold:
                self._on[i] = False
                continue
            self._on[i] = True
            powers[i] = self._power(kpa)

        return powers
