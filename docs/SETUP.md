# Development environment setup

## 1. Python environment

Python 3.12 is required -- upstream `dex-retargeting` requires `< 3.13`.

### Where the environment must live

**Create it at `<repository>/.venv`.** The launch file looks for
`.venv/bin/python` there, which is why nothing has to be activated before
`ros2 launch`.

If it lives anywhere else -- a conda environment, or a venv under a different
name -- the launch file falls back to `python3` on `PATH`. That is usually the
system Python, and the nodes then fail to import `v5s_teleop`. **It fails at
import time, not at launch time**, so the message is easy to misread.

To use a different interpreter, say so explicitly:

```bash
ros2 launch launch/v5s.launch.py hands:=left python:=/path/to/bin/python
```

### Standard venv (no extra tooling)

```bash
python3.12 -m venv .venv
.venv/bin/pip install --upgrade pip

# CPU torch first, otherwise pip pulls ~3 GB of CUDA libraries
.venv/bin/pip install --index-url https://download.pytorch.org/whl/cpu torch
.venv/bin/pip install -e .
```

`dex-retargeting` is on PyPI, so this pulls everything.

**Install the CPU torch first.** `dex-retargeting` requires torch, and the
default PyPI wheel brings the full CUDA stack (`nvidia-cublas`, `nvidia-cudnn`
and so on). Nothing here uses a GPU. Measured on a clean install:

```
plain  pip install -e .                     6.0 GB   (3.2 GB of it CUDA)
CPU wheel first, then pip install -e .      about 1 GB
```

`uv` handles this with `--torch-backend=cpu`, shown below.

### If `venv` fails

Some Ubuntu images ship Python without `ensurepip`, and `python3 -m venv` then
fails. Either install the package:

```bash
sudo apt install python3.12-venv
```

or use [`uv`](https://docs.astral.sh/uv/), which needs no sudo:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv --python 3.12 .venv
VIRTUAL_ENV=.venv uv pip install --torch-backend=cpu -e .
```

### Why venv is recommended

A venv does not bring its own interpreter -- it points at the system one:

```
.venv/pyvenv.cfg
  home = /usr/bin          <- the system Python 3.12
```

`rclpy` is a compiled extension built against that same interpreter
(`_rclpy_pybind11.cpython-312-x86_64-linux-gnu.so`), so inside a venv the ABI
matches automatically. That is the whole reason, and it is worth one sentence
because the alternative can fail in confusing ways.

### Conda

Conda works. It just brings its own Python and its own `libstdc++`, so two
things have to line up that a venv gets for free:

- **The Python minor version must match the ROS2 distribution** -- 3.12 for
  Jazzy, 3.10 for Humble. `rclpy` is compiled for one minor version only; with
  any other, `import rclpy` fails outright.
- **`libstdc++` conflicts.** `rclpy` links the system `libstdc++`, while an
  activated conda environment puts its own first. The symptom is
  `GLIBCXX_... not found`, or a crash at import.

Conda also is not detected automatically, so pass the interpreter:

```bash
conda create -n v5s python=3.12 && conda activate v5s
pip install --index-url https://download.pytorch.org/whl/cpu torch
pip install -e .
ros2 launch launch/v5s.launch.py hands:=left python:=$(which python)
```

> Neither issue applies to the core (retargeting, MANO conversion, haptic
> mapping, bridge client) -- none of it imports `rclpy`, so tuning, testing and
> the visualization tools all work under conda regardless. Only the two nodes in
> `src/v5s_teleop/ros2/` are affected.

> `dex-retargeting` imports `torch` without declaring it as a dependency. Our
> `pyproject.toml` declares it, so any of the commands above install it.
> The CPU build of torch is sufficient; nothing here uses a GPU.

To install upstream from a local source tree instead of PyPI, add its path
before `-e .`:

```bash
VIRTUAL_ENV=.venv uv pip install --torch-backend=cpu ./dex-retargeting-main -e .
```

Upstream is used unmodified either way.

## 2. MANUS SDK

**Not included in this repository** -- redistribution is not permitted (see
[`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)). Obtain it from [`MANUS`](https://docs.manus-meta.com/3.2.0/Resources/#manus-core-31-integrations) and
unpack it.

`MANUS_SDK` must point at the folder holding `include/` and `lib/`, which in the
MANUS distribution is one level in -- **not** the top of the archive:

```
<wherever you unpacked it>/
  license/
  ROS2/
  SDKClient_Linux/
    ManusSDK/            <- MANUS_SDK points here
      include/
      lib/               (libManusSDK_Integrated.so)
    SDKClient_Linux.out  (the client used for calibration)
  SDKClient_Windows/
```

Unpacking under `external/` in this repository matches the built-in default, and
that path is gitignored so the SDK is never committed:

```bash
mkdir -p external && tar -xf <manus-sdk-archive> -C external/
make
```

Otherwise pass the path:

```bash
make MANUS_SDK=/path/to/SDKClient_Linux/ManusSDK
```

The default is `external/ManusSDK_v3.1.1/SDKClient_Linux/ManusSDK`. It contains
the version number, so a different SDK version needs `MANUS_SDK=` or a renamed
folder. `make` checks for `$(MANUS_SDK)/include` and stops with the path it
tried if it is not there.

## 3. ROS2 and the hand driver

Match the distribution the hand driver uses. `rclpy` comes from the ROS2
distribution, not from pip:

```bash
source /opt/ros/jazzy/setup.bash
source ~/<driver workspace>/install/setup.bash
```

The driver package is [`allegro_hand_ros2_V5_Sense`](https://github.com/Wonikrobotics-git/allegro_hand_ros2_V5_Sense).

## 4. Verify

No hardware is needed for this:

```bash
make            # builds the bridge and three diagnostic tools
make test       # smoke tests plus the left/right symmetry check
```

`make test` checks that both configs build, that the joint reordering is correct,
that the core imports without `rclpy`, that the output stays inside the URDF
limits, that the wire format round-trips, and that each tactile channel drives
its own finger.

## Verified configuration

| | |
|---|---|
| Ubuntu | 24.04 |
| ROS2 | Jazzy |
| Python | 3.12.3 |
| MANUS SDK | 3.1.1 (Core Integrated mode) |
| dex-retargeting | 0.5.0, unmodified |
| pinocchio | 4.1.0 |
| numpy | 2.5.2 |
| pyzmq | 27.1.0 |
| CAN adapter | PEAK PCAN-USB, one per hand |
