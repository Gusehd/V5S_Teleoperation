# Third-Party Notices

External components used by this project and their license notices. **Add a line
here whenever a dependency is added** -- at release time this file is the record
of license compliance.

Each entry gives the name, source, license, purpose and how it is included.
"Included as" is either `dependency` (installed only, source not present) or
`bundled` (source is in this repository).

---

## Runtime dependency

| Component | Source | License | Purpose | Included as |
|---|---|---|---|---|
| dex-retargeting | github.com/dexsuite/dex-retargeting | MIT | Hand retargeting optimizer (DexPilot) | dependency |

### MIT notice (dex-retargeting)

The MIT license requires that "the copyright notice and this permission notice"
be included in copies. The upstream LICENSE is reproduced in full below.

```
The MIT License (MIT)

Copyright (c) 2023 Yuzhe Qin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

**Lineage**, as stated by the upstream README:

```
DexPilot (Handa, Van Wyk, Yang, Fox et al., ICRA 2020)   methodology
   └─> AnyTeleop (Qin et al., RSS 2023)                   system
          └─> dex-retargeting (MIT)                       the implementation we use
```

> "This repository is derived from the AnyTeleop Project"
> "The `DexPilotOptimizer` is crafted using insights from DexPilot"

Upstream asks that **AnyTeleop** be cited (`CITATION.cff`). We also cite
**DexPilot**, the source of the methodology.

**On our extensions.** Upstream sources are not modified. Project-specific logic
(applying a measured pinch projection distance, the self-collision penalty, and
the wrist reference frame correction) is implemented in
`src/v5s_teleop/retarget/` by **subclassing** upstream's public classes.
Upstream exists only as a pip dependency and is not bundled here.

---

## Python runtime dependencies

Verified directly from the license files of the installed packages (2026-08-25),
from the bundled license text rather than declared metadata. **Re-check when
bumping versions.**

| Package | Version | License | Purpose |
|---|---|---|---|
| `dex-retargeting` | 0.5.0 | MIT | Retargeting optimization core (unmodified dependency) |
| `pin` (Pinocchio) | 4.1.0 | BSD-2-Clause | Forward kinematics and Jacobians |
| **`nlopt`** | 2.11.0 | **LGPL-2.1-or-later** | SLSQP optimization -- see the note below |
| `numpy` | 2.5.2 | BSD-3-Clause | Numerics |
| `scipy` | 1.18.0 | BSD-3-Clause | (transitive) |
| `torch` | 2.13.0 | BSD-3-Clause | Required by upstream (the CPU build is sufficient) |
| `pyzmq` | 27.1.0 | BSD-3-Clause | Glove bridge transport |
| `PyYAML` | 6.0.3 | MIT | Config files |
| `lxml` | 6.1.2 | BSD-3-Clause | URDF parsing (transitive) |
| `anytree` | 2.13.0 | Apache-2.0 | (transitive) |

### Note on nlopt (LGPL)

From the license text shipped with the package:

> The compiled NLopt library, i.e. the combined work of all of the included
> optimization routines, is licensed under the conjunction of all of these
> licensing terms. Currently, the most restrictive terms are for the code in
> the "luksan" directory, which is licensed under the GNU Lesser General
> Public License (GNU LGPL), version 2.1 or later.
> **That means that the compiled NLopt library is governed by the terms of the LGPL.**

Upstream `dex-retargeting` imports `nlopt` directly, and the SLSQP solver we use
(`nlopt.LD_SLSQP`) comes from it. **This is a required, non-substitutable path.**

The LGPL does not restrict commercial use, but it carries obligations:

- **notice** that the library is used
- **inclusion** of the LGPL text
- the user must be able to **replace** the library -- keeping it as a pip
  dependency satisfies this. **Static linking into a binary, or vendoring the
  source, makes the obligations considerably heavier.**

---

## C++ build dependencies (glove bridge and diagnostic tools)

What the `Makefile` links against. Only the sources are published; **no binaries
are distributed.**

| Library | Version | License | Purpose |
|---|---|---|---|
| `libzmq` | 4.3.5 | **MPL-2.0** | Bridge to retargeting transport |
| `libManusSDK_Integrated` | 3.1.1 | **Proprietary** | Glove stream and vibration (see "Hardware SDK") |

MPL-2.0 is **file-level copyleft**. We link against `libzmq` dynamically and do
not modify it, so it has no effect on the license of our code. Modifying it would
require publishing those files under the MPL.

---

## ROS2

`rclpy`, `sensor_msgs` and `rcl_interfaces` come from the ROS2 distribution
(Apache-2.0). They are not bundled here.

The **hand driver** `allegro_hand_ros2` (Wonik Robotics, **BSD**) is also a
runtime dependency -- it is what receives `joint_cmd` and publishes
`joint_states` and `tactile_sensors`. It is not bundled; users install it
separately.
<https://github.com/Wonikrobotics-git/allegro_hand_ros2_V5_Sense>

---

## Hardware SDK

| Component | Vendor | License | Included as |
|---|---|---|---|
| MANUS SDK | MANUS Meta | Proprietary (licensed to us) | **Not included** -- users install it themselves |

> MANUS SDK binaries and headers are **not in this repository.** Redistribution
> is not permitted and the SDK is large. The setup guide covers installation
> instead.

---

## References (no code used, methodology cited)

Published papers cited as the basis of the implementation. No code was taken
from them.

| Reference | Where it is cited |
|---|---|
| Handa, Van Wyk et al., *DexPilot*, ICRA 2020 (arXiv:1910.03135) | Retargeting cost function and pinch projection |
| Qin, Yang et al., *AnyTeleop*, RSS 2023 (arXiv:2307.04577) | Design basis of dex-retargeting |

---

## Deliberate exclusions (not used -- recorded for the record)

Components **deliberately excluded** during license review, recorded so that the
reasoning is available later.

| Component | Reason for exclusion |
|---|---|
| GeoRT (facebookresearch) | CC-BY-NC -- non-commercial only, incompatible with commercial distribution |
| A third-party teleoperation repository with no license file | No LICENSE file means all rights reserved |
| AnyDexRT implementations | Source not published |
