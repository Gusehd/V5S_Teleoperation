# Development environment setup

## 1. Python environment

Python 3.12 is required -- upstream `dex-retargeting` requires `< 3.13`.

The standard `venv` module can fail without `ensurepip` on some Ubuntu images.
**`uv` works without sudo:**

```bash
uv venv --python 3.12 .venv
VIRTUAL_ENV=.venv uv pip install --torch-backend=cpu -e .
```

> `dex-retargeting` imports `torch` without declaring it as a dependency. Our
> `pyproject.toml` declares it, so the command above installs it too.
> The CPU build of torch is sufficient; nothing here uses a GPU.

To install upstream from a local source tree instead of PyPI, add its path
before `-e .`:

```bash
VIRTUAL_ENV=.venv uv pip install --torch-backend=cpu ./dex-retargeting-main -e .
```

Upstream is used unmodified either way.

## 2. MANUS SDK

**Not included in this repository** -- redistribution is not permitted (see
[`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)). Obtain it from MANUS,
unpack it somewhere, and point the build at it:

```bash
make MANUS_SDK=/path/to/ManusSDK
```

The default path is `external/ManusSDK_v3.1.1/SDKClient_Linux/ManusSDK`.

Two udev details are easy to miss:

- The license dongle appears in `lsusb` as **"Zalman Tech"** (a SenseLock
  device), not under the MANUS name, so it is easy to conclude it is absent.
- Default `hidraw` permissions are 0600 (root only). Without a udev rule, running
  as a normal user produces a "No compatible license found" warning and **no
  skeleton stream is generated**.

Both rules are installed by the scripts in `tools/`:

```bash
sudo bash tools/install_license_udev.sh
sudo bash tools/install_glove_udev.sh
```

## 3. ROS2 and the hand driver

Match the distribution the hand driver uses. `rclpy` comes from the ROS2
distribution, not from pip:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hand_ws/<driver workspace>/install/setup.bash
```

The driver package is `allegro_hand_controllers`. Each hand needs its own CAN
interface, at 1 Mbit/s:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
ip -details link show can0 | grep "can state"     # expect ERROR-ACTIVE
```

`ERROR-PASSIVE` means the bus has backed off after accumulating receive errors.
Bring the interface down and up again; if it persists, check termination,
wiring and hand power before running anything.

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
