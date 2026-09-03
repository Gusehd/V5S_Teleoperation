# Runbook

**Follow this and it reproduces.** Left hand = `allegroHand_0`, right hand =
`allegroHand_1`.

In every terminal, first:

```bash
source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
```

After editing anything in C++ (the bridge or the diagnostic tools) you **must
rebuild**. A stale binary keeps running and looks like it is working:

```bash
make            # rebuilds only what changed
make test       # smoke tests plus the symmetry check (no hardware, a few seconds)
```

---

## 0. One command (a single terminal) -- normally use this

```bash
source /opt/ros/jazzy/setup.bash
source ~/hand_ws/<driver workspace>/install/setup.bash
cd ~/V5S_Teleop && make          # only after editing C++

ros2 launch launch/v5s.launch.py hands:=both
```

**The venv does not need activating** -- the launch file finds the repository's
`.venv/bin/python` itself.

Arguments select what comes up:

| Argument | Values | Default |
|---|---|---|
| `hands` | `left` / `right` / `both` | `left` |
| `haptics` | `same` / `none` / `left` / `right` / `both` | `same` (follows `hands`) |
| `driver` | include the hand drivers | `true` |
| `bridge` | include the glove bridge | `true` |
| `dry_run` | emit no `joint_cmd` or vibration | `false` |
| `diag` | teleop latency instrumentation | `false` |
| `startup_delay` | seconds to wait for the bridge | `6.0` |
| `can_left` / `can_right` | CAN ports | `can0` / `can1` |
| `python` | Interpreter for the nodes | `<repo>/.venv/bin/python`, then `python3` |

```bash
ros2 launch launch/v5s.launch.py hands:=right                 # right hand, with haptics
ros2 launch launch/v5s.launch.py hands:=right haptics:=none   # right hand teleop only
ros2 launch launch/v5s.launch.py hands:=both dry_run:=true    # both hands, hardware still
ros2 launch launch/v5s.launch.py hands:=both driver:=false    # drivers started separately
ros2 launch launch/v5s.launch.py hands:=left haptics:=both    # left teleop, both gloves buzz
```

Log lines are prefixed with the process name, like `[teleop_left-3]`.
**One `Ctrl+C` brings everything down.** The bridge zeroes vibration on exit.

> If the bridge dies, everything is shut down. This avoids leaving teleop running
> with no glove behind it.

The argument list is always available:

```bash
ros2 launch launch/v5s.launch.py --show-args
```

> Sections A to E below start each process **by hand**. Use them to restart a
> single process, or to narrow down which stage is misbehaving.

---

## A. Teleoperation only (three terminals)

### Terminal 1 -- hand driver

```bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=left NUM:=0 CAN_DEVICE:=can0
```

Check: `ros2 topic list | grep allegroHand_0` should show `joint_cmd`,
`joint_states` and `tactile_sensors`.

### Terminal 2 -- glove bridge

```bash
./bridge_cpp/manus_bridge
```

**Check these four lines as soon as it starts:**

```
[bridge] publishing left  on tcp://127.0.0.1:5555
[bridge] publishing right on tcp://127.0.0.1:5557
[bridge] running -- Ctrl+C to stop
[bridge] left 113 Hz | nodes=25 | 1234 frames | undelivered 0 | unidentified 0 | 0 recoveries | gloves L=0x1EC9928C R=0x0
```

| Field | Normal | If not |
|---|---|---|
| `unidentified` | **0** | Side identification failed. Every frame is discarded and **the hand will not move** |
| `gloves L=` | **not 0** | The MANUS landscape callback never arrived |
| `Hz` | around 120 wired, around 113 wireless | Check the glove connection and power |
| `undelivered` | 0 | A subscriber is not keeping up |

With both gloves attached the rate is printed per hand:
`left 113 / right 112 Hz`.

### Terminal 3 -- teleop

```bash
python -m v5s_teleop.ros2.teleop_node --num 0
```

Run it with no flags. The settled values live in the config file:

```
scaling 1.5   offset [-0.06, 0.0, -0.169]   shape_weight 0.2
```

Normal log:

```
[v5s_teleop_left] publishing allegroHand_0/joint_cmd  QoS=RELIABLE depth=10 lifespan=infinite
[v5s_teleop_left] 120 Hz  s=1.50 o=[-0.06, 0.0, -0.169] w=0.2 | index ... | middle ... | thumb ...
```

**On a first run, or after changing the config, check the joint angles with
`--dry-run` first.** That is the hardware safety rule.

---

## B. Teleoperation plus haptic feedback (four or five terminals)

Terminal 1 from section A is unchanged. **Only terminal 2 differs.**

### Terminal 2 -- bridge, `--haptics` required

```bash
./bridge_cpp/manus_bridge --haptics tcp://127.0.0.1:5556
```

**Without this flag no vibration goes out.** It is off by default so that it
cannot affect teleoperation -- without the flag the socket is never even opened.

One flag opens **both per-hand sockets**. Change the right-hand address with
`--haptics-right`:

```
[bridge] haptic input left  tcp://127.0.0.1:5556
[bridge] haptic input right tcp://127.0.0.1:5558
```

The haptics node connects to the socket matching `--hand` on its own. **There is
no need to pass `--glove-id`** -- the socket identifies the hand, and the bridge
sends to that hand's glove.

### Terminal 3 -- teleop (same as section A)

```bash
python -m v5s_teleop.ros2.teleop_node --num 0
```

### Terminal 4 -- haptics

```bash
python -m v5s_teleop.ros2.haptics_node --num 0
```

When it works, **terminal 2** shows this -- proof that vibration actually went
out:

```
[bridge]   haptic received 12 / errors 0
```

The haptics node's own log (once per second):

```
[v5s_haptics_left] 60Hz [linear 5~40kPa g=1.0] | tip kPa  0.0  22.7  0.0  0.0 | vibration 0.00 0.56 0.00 0.00
```

### Terminal 5 (optional) -- on-screen visualization

```bash
python tools/haptics_viz.py --num 0
```

Draws pressure and vibration large, at 15 Hz. **It only reads, so it is safe to
run alongside the haptics node.**

```
INDEX
  pressure   22.7 kPa  ██████████████████████······
  vibration  0.56      ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬···
```

### Shutdown order

**Reverse order (5, 4, 3, 2, 1).** Always stop the bridge with `Ctrl+C` -- the
code that zeroes vibration on exit lives there. A forced kill leaves the last
strength latched in the glove.

---

## C. Stutter and latency diagnosis

```bash
python -m v5s_teleop.ros2.teleop_node --num 0 --diag 2>&1 | tee /tmp/diag.log
```

The per-layer maximum delay prints every second:

```
diag max[ms]  glove-send 8.7 | recv 9.3 | tick 1.8 | solve 1.7 | publish 0.1 | driver 3.0
```

| What spikes | Meaning |
|---|---|
| send and recv only, tick normal | The data did not arrive -- glove or bridge |
| tick spikes too | Our own process stalled |
| tick and driver together | A system-level event stalled both |
| publish | The publishing stage |
| solve | The optimizer |

**Keep the bridge log too** -- separating the layers needs both sides:

```bash
./bridge_cpp/manus_bridge 2>&1 | tee /tmp/bridge.log
```

Baselines (wired, left hand):

```
send 8.7ms   recv 9.3ms   tick 1.8ms   solve 1.7ms   publish 0.1ms   driver 3.0ms
SDK's own report: average frame duration 8.301ms, average data delay 4.105ms
```

Wireless, both gloves on one dongle:

```
send/recv max 24-34ms (2.5-3% of packets discarded), data delay 4.0-4.2ms
```

The average latency matches wired; only the maximum inter-frame gap grows. At
113 Hz the period is 8.8 ms, so 25-34 ms means three or four frames are missed
at a time. The low-pass filter absorbs this.

> The first one or two seconds of `--diag` output are unreliable -- DDS
> discovery is still settling and the `driver` column will show a large false
> gap.

---

## D. Adjusting parameters at runtime

Change them with the node running. **The node name carries the hand.**

```bash
ros2 param set /v5s_teleop_left shape_weight 0.15
ros2 param set /v5s_teleop_left scaling_factor 1.4
ros2 param set /v5s_teleop_left low_pass_alpha 0.15
ros2 param set /v5s_teleop_left wrist_offset "[-0.05, 0.0, -0.169]"   # rebuilds, ~1s pause

ros2 param set /v5s_haptics_left gamma 0.7
ros2 param set /v5s_haptics_left mode step
ros2 param set /v5s_haptics_left min_kpa 8.0
```

> Changing a value at runtime leaves **the influence of the previous solution**
> behind (the `norm_delta` regularization). Opening and closing the hand fully
> washes it out. **Confirm the final value once more after a restart.**

What each value means and why: [`PARAMETERS.md`](PARAMETERS.md).

---

## E. Both hands

One bridge serves both hands. **The ports differ.**

```bash
# terminal 2
./bridge_cpp/manus_bridge --haptics tcp://127.0.0.1:5556
#   left  tcp://127.0.0.1:5555
#   right tcp://127.0.0.1:5557   (change with --right)

# terminals 3 and 4
python -m v5s_teleop.ros2.teleop_node --hand left  --num 0
python -m v5s_teleop.ros2.teleop_node --hand right --num 1

# terminals 5 and 6   (no --glove-id needed; the per-hand socket handles it)
python -m v5s_teleop.ros2.haptics_node --hand left  --num 0
python -m v5s_teleop.ros2.haptics_node --hand right --num 1
```

Port summary:

| | Left | Right |
|---|---|---|
| Glove stream (PUB) | 5555 | 5557 (`--right`) |
| Haptic input (PULL) | 5556 | 5558 (`--haptics-right`) |

Node names split into `/v5s_teleop_left` and `/v5s_teleop_right`.

Each hand needs its own CAN interface (`can0` and `can1`); see
[`SETUP.md`](SETUP.md).

**Without a second robot hand**, run the right side with `--dry-run` to check the
stream only:

```bash
python -m v5s_teleop.ros2.teleop_node --hand right --num 1 --dry-run --diag
```

To check which physical glove arrives on which port, shake one glove at a time:

```bash
python tools/check_lr_mapping.py
```

---

## F. Diagnostic tools

What each one is for: [`../tools/README.md`](../tools/README.md).

```bash
python tools/glove_live.py                   # fingertip coordinates and range of motion
python tools/tactile_live.py --num 0         # raw values of all 16 tactile channels
python tools/haptics_viz.py --num 0          # pressure to vibration (for recording)
python tools/retarget_live.py                # retargeting result
python tools/recommend_scaling.py --glove-id 0x...   # per-user scale
python tools/check_lr_symmetry.py            # left/right config symmetry (no hardware)
python tools/check_lr_mapping.py             # which glove is on which port
./tools/manus_diag                           # MANUS SDK connection diagnosis
./tools/manus_nodes                          # glove node layout
```

---

## G. Common problems

| Symptom | Cause | What to do |
|---|---|---|
| Hand does not move at all | The bridge's `unidentified` count is rising | Side identification failed; check the bridge status line |
| Hand does not move at all | QoS mismatch | The driver subscribes `RELIABLE`; never publish `BEST_EFFORT` |
| Hand does not move, but frames arrive | The glove is not calibrated | The skeleton freezes with no error. See `SETUP.md` |
| Zero glove frames | Bridge not running, or port mismatch | Check that `--hand` matches the bridge port |
| `No compatible license found`, and no `is connected as MetaglovePro Dongle` line | The dongle's USB interfaces lost their driver binding | **Unplug and replug the dongle.** See below |
| No vibration | The bridge has no `--haptics` | It is off by default |
| Vibration will not stop | The bridge was force-killed | Always stop it with `Ctrl+C` |
| Pinch never closes | `eta1` is at the upstream default (`1e-4`) | Ours is `0.027`; check the config file is being used |
| Only one phalanx bends | `shape_weight` is 0 | The config ships 0.2 |
| Cannot make a fist | `wrist_offset` x | See `PARAMETERS.md` |

---

## H. The dongle stops being detected

Symptom -- the bridge starts, but:

```
[warning] No compatible license found. Please connect a license with the SDK component.
[bridge] waiting for stream (14 s left)
```

and the line `0xE9767DCA is connected as MetaglovePro Dongle` never appears. The
glove LEDs can still be blue: that only means the gloves are paired **to the
dongle**, not that the host can see it.

Confirm it with:

```bash
lsusb | grep 3325                                    # the dongle is present on USB
for d in /sys/bus/usb/devices/*/; do
  [ "$(cat $d/idVendor 2>/dev/null)" = "3325" ] && ls -l $d*:*/driver 2>&1
done
```

If the interfaces have **no driver bound**, that is the cause: CoreLite's HIDAPI
bridge detaches the kernel driver, and a process that dies abnormally never
reattaches it. No hidraw node is created, so the SDK cannot read the license.

**Fix: unplug and replug the dongle.** Or rebind it in software (substitute the
device path found above):

```bash
echo '3-11.2' | sudo tee /sys/bus/usb/drivers/usb/unbind
sleep 2
echo '3-11.2' | sudo tee /sys/bus/usb/drivers/usb/bind
```

The udev rules give the new hidraw node its permissions automatically, so
nothing else is needed.

> This is why the bridge should always be stopped with `Ctrl+C` rather than
> killed. A forced kill leaves the dongle in this state.
