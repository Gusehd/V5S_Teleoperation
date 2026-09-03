# Parameter reference

Every value an operator can touch. Values **with a measured basis are separated
from values chosen by preference** -- the former should not be changed unless the
hardware does.

---

## At a glance

| Parameter | Current | Kind | Changeable at runtime |
|---|---|---|---|
| `scaling_factor` | 1.5 | per user | yes |
| `wrist_offset` | `[-0.06, 0, -0.169]` | z geometric / x preference | yes (~1 s rebuild) |
| `shape_weight` | **0.2** | preference | yes |
| `low_pass_alpha` | 0.2 | preference | yes |
| `eta1` | 0.027 | **measured constant** | no, restart |
| `eta2` | 0.03 | measured constant | no |
| `project_dist` / `escape_dist` | 0.03 / 0.05 | measured constant | no |
| `collision_weight` | 0.0 (off) | preference | no (CLI) |
| `joint_limit_overrides` | `{}` (none) | safety guard | no |
| `min_kpa` / `max_kpa` / `gamma` | 5 / 40 / 1.0 | preference (haptics) | yes |
| `mode` | linear | preference (haptics) | yes |

These are the values in both the left and right yml files, so running with no
flags uses exactly this.

---

## 1. Retargeting

File: `src/v5s_teleop/configs/v5s_{left,right}_dexpilot.yml`
Schema: `v5s_teleop.retarget.builder.RetargetingSpec`

### 1.1 `scaling_factor` -- human hand to robot hand scale

```yaml
scaling_factor: 1.5
```

The human fingertip vectors are multiplied by this to form the robot's targets.
**It differs per user.**

Recompute when the user changes:

```bash
python tools/recommend_scaling.py --glove-id <glove id>
```

- **Larger**: small movements produce large robot motion, at the cost of more
  unreachable targets
- **Smaller**: more stable, but the hand has to move further

```bash
ros2 param set /v5s_teleop_left scaling_factor 1.4
```

### 1.2 `wrist_offset` -- virtual wrist position `[x, y, z]` (m)

```yaml
wrist_offset: [-0.06, 0.0, -0.169]
```

This hand's URDF has **no link corresponding to a wrist.** The `palm_link` origin
is at the finger bases (the knuckles). Retargeting works from wrist-relative
vectors, so we establish that reference point ourselves. `retarget/builder.py`
adds a fixed link to a copy of the URDF at runtime -- **the original is never
modified.**

### 1.3 `shape_weight` -- shape matching strength

```yaml
shape_weight: 0.2      # 0 turns it off
```

All ten DexPilot targets are **fingertip positions**, which leaves one degree of
freedom undetermined across a finger's four joints. More than eight joint
combinations produce the same fingertip position. The result is that curling a
finger **bends one phalanx while the rest stay stiff.**

This term matches **which way each segment points** (its unit vector) against the
human. Directions rather than positions, because the robot's segments are not a
scaled copy of a human's (on the index finger: proximal 1.01x, intermediate
1.59x, distal 1.91x).

- **Useful range 0.0 to 0.2.** Beyond that, fingertip accuracy is gone
- The thumb is excluded (see section 4)

```bash
ros2 param set /v5s_teleop_left shape_weight 0.2
```

### 1.4 `low_pass_alpha` -- output smoothing

```yaml
low_pass_alpha: 0.2
```

`output = alpha * new + (1 - alpha) * previous`. Smaller is smoother but adds
latency. 0.2 at 120 Hz is a time constant of about 42 ms.

### 1.5 `eta1` / `eta2` / `project_dist` / `escape_dist` -- pinch

```yaml
eta1: 0.027          # fingertip target distance once considered touching
eta2: 0.03           # separation to hold between other fingers during a pinch
project_dist: 0.03   # closer than this counts as a pinch (projection on)
escape_dist: 0.05    # wider than this turns projection off (hysteresis)
```

**`eta1 = 0.027` is a measured value.** The `link_*_tip` frame origins sit
**inside** the fingertip mesh (9.8 mm from the surface on the fingers, 9.5 mm on
the thumb). Across 1610 near-contact thumb-to-index poses, the tip-origin
distance at actual mesh contact was **26.9 +/- 8.1 mm**.

> The upstream `dex-retargeting` default is `1e-4`. That distance requires the
> fingertips to overlap by 27 mm, so **the pinch never closes.** Injecting this
> value is the reason we assemble the classes directly instead of using
> upstream's config loader.

### 1.6 `collision_weight` / `collision_link_pairs` -- self-collision

```yaml
collision_weight: 0.0        # 0 turns it off entirely (default)
collision_link_pairs:
  - [ "link_3_0_tip", "link_7_0_tip",  0.0343 ]   # index-middle
  - [ "link_7_0_tip", "link_11_0_tip", 0.0349 ]   # middle-ring
```

Turn it on only when fingers are visibly colliding.

```bash
python -m v5s_teleop.ros2.teleop_node --num 0 --collision-weight 1000
```

### 1.7 `joint_limit_overrides` -- abduction clamp

```yaml
joint_limit_overrides: {}    # empty
```

This hand's abduction range is 30-55 degrees, comparable to a human hand (an
earlier hand had 160-180 degrees). **The hardware already acts as the clamp, so
this is left empty.** Narrow it if fingers splay sideways during a pinch.

### 1.8 Values not to touch

```yaml
normal_delta: 0.004    # upstream default -- regularization strength
huber_delta:  0.02     # upstream default -- Huber loss threshold
```

These are upstream's tuned values; do not change them without a reason. Omitting
them from the yml uses these defaults.

---

## 2. Haptics

File: `src/v5s_teleop/configs/haptics.yml`
Schema: `v5s_teleop.haptics_map.HapticMapping`

**This pipeline is completely separate from teleoperation.** Turning haptics off
does not affect teleoperation.

The hand has 16 tactile channels but the glove has only one vibration channel per
finger (five in total), so **only fingertip pressure is sent, 1:1** (Pinky is
always 0 on this four-finger hand).

### `mode` -- linear or step

```yaml
mode: linear
```

- **linear**: vibration varies continuously with pressure
- **step**: each band emits a fixed strength. People notice **the moment a band
  changes** far more readily than a small difference in strength

### `min_kpa` -- dead zone

```yaml
min_kpa: 5.0
```

The sensors read slightly above zero even with no contact (operator measurement).
**At 0 the glove buzzes constantly.** Raise it and a firm press is needed; lower
it and light contact registers, at the cost of noise.

### `max_kpa` -- saturation point

```yaml
max_kpa: 40.0
```

Vibration is maximum at this pressure. The sensor range is 0-400 kPa, but **the
useful range is 0-40** (operator measurement). The step-mode band edges scale
with this value.

### `gamma` -- response curve

```yaml
gamma: 1.0     # vibration = normalized_pressure ** gamma
```

- `0.5` rises early -- even a graze is felt clearly (sensitive)
- `1.0` proportional to pressure
- `2.0` needs a firm press to respond (dull)

```bash
ros2 param set /v5s_haptics_left gamma 0.7
ros2 param set /v5s_haptics_left mode step
```

### Internal constants kept out of the config

Held inside `haptics_map.py` to keep the surface a user has to touch small.

```
_MOTOR_FLOOR  0.12   minimum strength at which the motor actually turns
_HYSTERESIS   1.4    once on, stay on until the signal drops by this factor
STEP_EDGES           step-mode band edges, as fractions of max_kpa
```

---

## 3. Run options (not parameters)

```bash
python -m v5s_teleop.ros2.teleop_node --num 0 --shape-weight 0.2
```

| Option | Default | Purpose |
|---|---|---|
| `--hand` | left | Select the left or right config |
| `--num` | 0 | The NUM in `allegroHand_<NUM>` -- left 0, right 1 |
| `--dry-run` | off | **Publishes nothing.** Logs the values only |
| `--diag` | off | Print per-layer maximum delay every second (stutter diagnosis) |
| `--qos-depth` | 10 | Publisher queue depth (the ROS2 default) |
| `--qos-lifespan-ms` | 0 (off) | Let DDS discard stale commands. **Unverified** |
| `--scaling` / `--wrist-offset` / `--shape-weight` / `--collision-weight` | config file | One-off overrides |

> **`BEST_EFFORT` publishing is not possible.** The driver
> (`allegro_node_grasp_0`) subscribes as `RELIABLE`, so the QoS would be
> incompatible and **the connection silently fails to form** (confirmed with
> `ros2 topic info -v`). The flag was removed entirely.

**On a first run with a new config, check the joint angles with `--dry-run`.**

---

## 4. Why the thumb is excluded from shape matching

```yaml
shape_segments:    # index, middle, ring x (proximal, intermediate, distal) = 9
```

The thumb (joints 12-15) is **deliberately excluded.** The case for adding it is
currently weak.

**Reasons**

1. **The segment correspondence is unverified.** Direction matching assumes
   "robot segment i corresponds to human segment i" as the same physical
   segment. The robot thumb is an opposed design whose first joint
   (`joint_12_0`) uses a completely different axis from the fingers. On top of
   that, **the mounting angle of the left and right thumbs differs by 10
   degrees**, so each side would need verifying separately.
2. **A far stronger term already handles the thumb's real job.** The pinch
   projection weight is 200-400 while the shape term is 0.2 -- a factor of 1000.
   A shape term on the thumb would mostly be ignored, or interfere
   unpredictably near the projection threshold.
3. **Nobody has complained about the thumb's shape.** The reported symptom was
   "the finger bends at only one phalanx", and that is fixed.

**Note -- `joint_13_0` saturating at its limit is not a problem**

In hardware logs `joint_13_0` sits at its upper limit (180 degrees) in 55% of
samples. This was suspected as a fault at one point, but **it is normal.**

`joint_13_0` is the thumb's **roll** joint -- it decides which way the thumb will
bend. With the thumb extended it is a rotation about its own axis, so the
fingertip does not move at all:

```
thumb extended, 102 to 180 deg      fingertip moves   0.0 mm
joint_14_0 = -50 deg (bent)         fingertip moves  84.8 mm
14=-50, 15=-60 (bent)               fingertip moves  95.8 mm
```

So while the thumb is unused this joint does not affect the objective, and the
regularization term parks it anywhere. **When it matters, it is mid-range:**

```
measured pinch pose    joint_13_0 = 134 deg    middle of the [102, 180] limit
```

The saturation is not "the optimizer could not get there" -- it is "there was no
reason to move".

> On the right hand this joint reads differently: the two URDFs' zero points are
> 180 degrees apart, so the same pose logs as 46 degrees. See the
> `joint_limit_overrides` comment in the right-hand config.
