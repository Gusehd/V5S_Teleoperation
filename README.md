# ✋ Teleoperation : Allegro Hand V5 Sense

Real-time teleoperation of a **[`Allegro Hand V5 Sense`](https://www.allegrohand.com/sub/product/p.php?idx=22)** from a
**MANUS Metaglove Pro Haptic glove**, with tactile feedback returned to the glove as
vibration.

```
[MANUS] --SDK--> [glove bridge, C++] --ZMQ--> [retargeting, Python] --rclpy--> [ROS2 hand driver]
                                                                                              |
                        [glove vibration] <--ZMQ-- [haptics node] <--------- tactile sensors --+
```

Both hands are supported and can run at the same time, wired or wireless.

## Requirements

| Item | Details |
|---|---|
| OS | Ubuntu (verified on 24.04) |
| ROS2 | Jazzy -- `rclpy` comes from the distribution, not pip |
| Python | 3.12 (upstream `dex-retargeting` requires < 3.13) |
| Hand driver | [`Allegro_hand_V5S_ros2`](https://github.com/Wonikrobotics-git/allegro_hand_ros2_V5_Sense) |
| [`MANUS SDK`](https://docs.manus-meta.com/3.2.0/Resources/) | Core 3 (tested on 3.1.1), obtained from MANUS under your own license |
| Hardware | MANUS Metaglove Pro Haptic, V5 Sense hand, one CAN interface per hand |

The MANUS SDK is **not** included here -- redistribution is not permitted. See
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

## Quickstart

**1. Install**

```bash
git clone <this repository>
cd V5S_Teleop
```

Create the virtual environment **at `.venv` inside the repository** (see the
note below on why the location matters.)

```bash
python3.12 -m venv .venv
.venv/bin/pip install --upgrade pip

# Install the CPU build of torch first -- see the note below
.venv/bin/pip install --index-url https://download.pytorch.org/whl/cpu torch
.venv/bin/pip install -e .
```

> **Install the CPU torch first.** `dex-retargeting` requires torch, and the
> default PyPI wheel pulls in about 3 GB of CUDA libraries. Nothing here uses a
> GPU, so that is wasted: a plain `pip install -e .` produces a 6 GB
> environment, against roughly 1 GB with the CPU wheel.

If `python3.12 -m venv` fails (some Ubuntu images ship without `ensurepip`),
[`uv`](https://docs.astral.sh/uv/) works without sudo:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh      # if uv is not installed
uv venv --python 3.12 .venv
VIRTUAL_ENV=.venv uv pip install --torch-backend=cpu -e .
```

> **A venv is recommended, not required.** Conda works too; see
> [`docs/SETUP.md`](docs/SETUP.md) for the two things that have to line up.
>
> **The environment must live at `<repository>/.venv`.** The launch file looks
> for `.venv/bin/python` there so that you do not have to activate anything. If
> the environment is elsewhere -- a conda env, or a venv under a different name
> -- the launch file silently falls back to `python3` on `PATH`, which is
> usually the system Python and will fail to import `v5s_teleop`.
>
> To use conda or another location, pass the interpreter explicitly:
>
> ```bash
> ros2 launch launch/v5s.launch.py hands:=left python:=/path/to/bin/python
> ```
>
> Only a plain CPU install is needed; nothing here uses a GPU.

**2. Build the C++ bridge**

```bash
make                                   # or: make MANUS_SDK=/path/to/ManusSDK
```

Rebuild after editing any `.cpp`. A stale binary still looks like it is working.

**3. Calibrate the glove** -- required, and it fails silently if skipped

Calibration is stored per glove ID, so **each glove must be calibrated
separately**. Without it the skeleton freezes in a fixed pose with no error, and
it looks like "connected but the hand will not move".

Calibration is done with the client that ships inside the MANUS SDK:

```
<SDK>/SDKClient_Linux/SDKClient_Linux.out
```

Build it with the `Makefile` in that same folder if the binary is not present.
Run it with the dongle connected and the glove powered on (its LED turns blue
once it is connected; white blinking means it is still waiting to pair, in which
case pair it first from the client's `P` menu).

Then, in the client:

| Key | Step |
|---|---|
| `C` | open the Glove Calibration menu |
| `H` | **switch hands** -- it starts on Left, so press this for the right glove |
| `S` | start calibration |
| `E` | run the current step; hold the posture shown on screen |
| `F` | finish once every step is done |

Check that the screen shows `Calibrating: <side>` and a non-zero `Glove:` id
before starting -- with `Glove: 0` the keys do nothing.

**Shut the client down completely before starting the bridge** -- the SDK allows
only one instance at a time.

**4. Adjust parameters for your hand**

The shipped values work, with one exception: **`scaling_factor` depends on the
size of your hand.** It is "how many times longer the robot's fingers are than
yours", so a new user is recommended to recompute it (the default is 1.5):

```bash
.venv/bin/python tools/recommend_scaling.py --glove-id <glove id>
```

Write the result into `src/v5s_teleop/configs/v5s_<hand>_dexpilot.yml`.

> **[`docs/PARAMETERS.md`](docs/PARAMETERS.md)** -- every adjustable value,
> split into retargeting (section 1) and haptics (section 2).

**5. Run**

```bash
# Run hand driver
source /opt/ros/jazzy/setup.bash
source ~/<driver workspace>/install/setup.bash

# example : Run hand driver & Start teleoperation
ros2 launch launch/v5s.launch.py hands:=left      # left hand, with haptics
ros2 launch launch/v5s.launch.py hands:=both      # both hands
```
> When running the controller, use NUM:=0 for the left hand and NUM:=1 for the right hand (default).

Examples:

```bash
# log joint angles without driving the hand
ros2 launch launch/v5s.launch.py hands:=left dry_run:=true

# teleoperation only, no vibration
ros2 launch launch/v5s.launch.py hands:=right haptics:=none

# the drivers are already running in their own terminals
ros2 launch launch/v5s.launch.py hands:=both driver:=false

# print the per-layer latency every second
ros2 launch launch/v5s.launch.py hands:=left diag:=true
```

Parameters can also be changed while it runs, without a restart. The node name
carries the hand:

```bash
ros2 param set /v5s_teleop_left scaling_factor 1.4
ros2 param set /v5s_teleop_left shape_weight 0.15
ros2 param set /v5s_haptics_left gamma 0.7
```

> Before driving the hardware with a config you have not used before, it is
> worth running `dry_run:=true` once and checking the joint angles in the log.

Full command reference: [`docs/RUNBOOK.md`](docs/RUNBOOK.md).

## Repository layout

| Path | Contents |
|---|---|
| `src/v5s_teleop/hand/` | Glove skeleton to MANO 21 points, wrist frame estimation |
| `src/v5s_teleop/retarget/` | Retargeting adapter -- our extensions on top of unmodified upstream |
| `src/v5s_teleop/bridge/` | Glove ZMQ stream client, vibration sender |
| `src/v5s_teleop/haptics_map.py` | Tactile pressure to glove vibration mapping |
| `src/v5s_teleop/configs/` | Per-hand retargeting configs (yml) |
| `src/v5s_teleop/ros2/` | The two ROS2 nodes (the only files that need `rclpy`) |
| `launch/` | One-command startup |
| `bridge_cpp/` | Glove bridge (C++, MANUS SDK) |
| `tools/` | Diagnostics and calibration helpers -- see [`tools/README.md`](tools/README.md) |
| `tests/` | Smoke tests -- `make test`, no hardware needed |
| `urdf/` | V5 Sense hand URDF |

The core (everything outside `src/v5s_teleop/ros2/`) does not import `rclpy`, so
retargeting can be tested without ROS. A smoke test enforces this.

## About the retargeting core

The retargeting optimization uses
[`dexsuite/dex-retargeting`](https://github.com/dexsuite/dex-retargeting) (MIT)
as an **unmodified dependency**. Upstream is neither forked nor vendored here.

What upstream does not provide is added by our code in
`src/v5s_teleop/retarget/`:

1. **Injecting the pinch projection distance (`eta1`).** Upstream does not expose
   this through its config and pins it at `1e-4`. That distance is only
   satisfied once the fingertip meshes interpenetrate, which on real hardware
   shows up as "the pinch never closes". We measure the fingertip frame's depth
   below the contact surface and pass the matching value.
2. **A virtual wrist frame (`wrist_offset`).** This hand's URDF has no link
   corresponding to a wrist or mounting face -- that is intentional in the
   design. Retargeting needs a wrist, so we establish that reference point
   ourselves.
3. **A self-collision penalty**, giving finger link pairs a minimum separation
   (off by default).
4. **Shape matching (`shape_weight`).** All upstream targets are fingertip
   positions, which leaves one degree of freedom undetermined across a finger's
   four joints -- the fingertip lands correctly while only one phalanx bends.
   This term matches the **direction** of each segment against the human hand to
   settle that freedom. It trades against fingertip accuracy, so the useful
   range is small.

The methodology follows DexPilot (Handa et al., ICRA 2020, arXiv:1910.03135).
The implementation is ours.

## Documentation

- [`docs/RUNBOOK.md`](docs/RUNBOOK.md) -- how to run it, terminal by terminal
- [`docs/PARAMETERS.md`](docs/PARAMETERS.md) -- every tunable value, what it does, how to tune it
- [`docs/SETUP.md`](docs/SETUP.md) -- development environment setup
- [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) -- dependency licenses
