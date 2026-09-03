# Tools

Diagnostic and calibration helpers. None of them are needed to run
teleoperation -- they exist to answer "is this part working?" when something is
wrong, and to set up a new user.

Python tools run from the repository root:

```bash
.venv/bin/python tools/<name>.py
```

C++ tools are built by `make` and run as `./tools/<name>`.

---

## Setup -- run these once

| Tool | What it does |
|---|---|
| `install_license_udev.sh` | udev rule for the MANUS license key (VID 1c57). Without it, `hidraw` is root-only and the SDK reports "No compatible license found" with no skeleton stream. `sudo bash tools/install_license_udev.sh` |
| `install_glove_udev.sh` | udev rule for the glove and dongle (VID 3325). Same problem, different device. `sudo bash tools/install_glove_udev.sh` |

## Per user -- run when the operator changes

| Tool | What it does |
|---|---|
| `recommend_scaling.py` | Computes `scaling_factor` from the MANUS calibration file: robot finger length divided by the user's, averaged over index, middle and ring. `--glove-id <glove_id>` selects the profile when more than one is stored |

## When something is wrong

Roughly in the order the data flows, so use them top to bottom to find the stage
that broke.

| Tool | Answers |
|---|---|
| `manus_diag` (C++) | Does the SDK connect at all? Prints dongle, license, glove model, and which streams produce events. Start here when the bridge sees nothing |
| `manus_nodes` (C++) | What is the glove's node layout? Dumps all 25 nodes with finger and phalanx. Needed if the glove model changes and the MANO mapping has to be redone |
| `glove_live.py` | Is the bridge publishing usable data? Shows fingertip coordinates and accumulated range of motion. **If everything reads 0.0 mm the glove is not calibrated** -- the skeleton freezes with no error |
| `check_lr_mapping.py` | Which physical glove arrives on which port? Shake one glove at a time and watch which row moves. Catches a swapped left/right mapping |
| `retarget_live.py` | Are the joint angles sensible? Runs the full retargeting and displays angles and pinch distances **without touching hardware**. `+`/`-` adjust scaling live |
| `tactile_live.py` | Are the tactile sensors reading? Raw values of all 16 channels with observed min and max. Use it to pick `min_kpa` and `max_kpa` |
| `haptics_viz.py` | Is the pressure-to-vibration mapping doing what you expect? Large bars at 15 Hz, using the same mapper as the haptics node. Read-only, so it is safe alongside it -- also useful for screen recording |

## Regression check

| Tool | What it does |
|---|---|
| `check_lr_symmetry.py` | Feeds a synthetic posture and its mirror to the two hands' configs and compares. The kinematics are an exact mirror, so the outputs must be too. Run by `make test`; needs no hardware |

See [`../docs/RUNBOOK.md`](../docs/RUNBOOK.md) for how these fit into a session,
and [`../docs/PARAMETERS.md`](../docs/PARAMETERS.md) for what the values mean.
