"""Bring the whole system up from one terminal.

    source /opt/ros/jazzy/setup.bash
    source ~/hand_ws/allegro_hand_ros2_V5_Sense-master/install/setup.bash
    ros2 launch launch/v5s.launch.py hands:=both

**The venv does not need to be activated** -- this file finds the repository's
`.venv/bin/python` directly. Only ROS and the driver workspace need sourcing.

What to bring up is chosen with arguments::

    hands:=left | right | both        which hand gets teleop + driver (default left)
    haptics:=same | none | left | right | both
                                      which hand gets vibration (default same as hands)
    driver:=true | false              include the hand driver (default true)
    bridge:=true | false              include the glove bridge (default true)
    dry_run:=true | false             do not emit joint_cmd (default false)
    diag:=true | false                teleop latency instrumentation (default false)
    python:=/path/to/bin/python       interpreter for the nodes (default:
                                      <repo>/.venv/bin/python, then python3)

Examples::

    ros2 launch launch/v5s.launch.py hands:=right                 right hand, with haptics
    ros2 launch launch/v5s.launch.py hands:=right haptics:=none   right hand teleop only
    ros2 launch launch/v5s.launch.py hands:=both  dry_run:=true   both hands, hardware still
    ros2 launch launch/v5s.launch.py hands:=both  driver:=false   drivers started separately
    ros2 launch launch/v5s.launch.py hands:=left  haptics:=both   left teleop, both gloves buzz

A single `Ctrl+C` brings everything down. The bridge zeroes vibration on exit.

Note: if the bridge dies, everything is shut down. This avoids leaving teleop
running with no glove behind it.
"""
from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription

#: Repository root. Assumes this file lives in launch/.
ROOT = Path(__file__).resolve().parent.parent

#: Hand numbering convention: left is allegroHand_0, right is allegroHand_1.
NUM = {"left": "0", "right": "1"}

#: CAN port per hand. With both hands they use separate buses.
CAN = {"left": "can0", "right": "can1"}

#: Driver package. It is `_controllers`, not `allegro_hand`.
DRIVER_PKG = "allegro_hand_controllers"

_HANDS = {"left": ["left"], "right": ["right"], "both": ["left", "right"]}


def _python(override: str = "") -> str:
    """The interpreter used for the nodes.

    Order: an explicit `python:=` argument, then the repository's own
    `.venv/bin/python`, then `python3` from PATH.

    The fallback is worth knowing about: with a conda environment, or a venv
    somewhere other than `<repository>/.venv`, this lands on the system Python
    and the nodes fail to import `v5s_teleop`. Pass `python:=` in that case.
    """
    if override:
        exe = Path(override).expanduser()
        if not exe.exists():
            raise RuntimeError(f"python:={override} does not exist")
        return str(exe)
    venv = ROOT / ".venv" / "bin" / "python"
    return str(venv) if venv.exists() else "python3"


def _setup(context, *_):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() in ("true", "1", "yes")

    hands_key = arg("hands").lower()
    if hands_key not in _HANDS:
        raise RuntimeError(f"hands must be left, right or both (got {hands_key!r})")
    hands = _HANDS[hands_key]

    hap_key = arg("haptics").lower()
    if hap_key == "same":
        haptics = list(hands)
    elif hap_key == "none":
        haptics = []
    elif hap_key in _HANDS:
        haptics = _HANDS[hap_key]
    else:
        raise RuntimeError(
            f"haptics must be same, none, left, right or both (got {hap_key!r})")

    py = _python(arg("python"))
    actions = [LogInfo(msg=(
        f"[v5s] teleop {'+'.join(hands)}"
        f" | vibration {'+'.join(haptics) if haptics else 'none'}"
        f" | driver {'included' if flag('driver') else 'excluded'}"
        f"{' | DRY RUN' if flag('dry_run') else ''}"))]

    # ── Hand drivers ─────────────────────────────────────────────────
    if flag("driver"):
        for hand in hands:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare(DRIVER_PKG), "/launch/allegro_hand.launch.py"]),
                launch_arguments={
                    "HAND": hand,
                    "NUM": NUM[hand],
                    "CAN_DEVICE": arg(f"can_{hand}"),
                }.items()))

    # ── Glove bridge (one instance serves both hands) ────────────────
    bridge_action = None
    if flag("bridge"):
        cmd = [str(ROOT / "bridge_cpp" / "manus_bridge")]
        if haptics:
            # This one flag opens both per-hand haptic sockets.
            cmd += ["--haptics", "tcp://127.0.0.1:5556"]
        bridge_action = ExecuteProcess(cmd=cmd, name="manus_bridge", output="screen")
        actions.append(bridge_action)

    # ── Teleop and haptics ───────────────────────────────────────────
    # Give the bridge time to bring the SDK up. ZMQ does not care about order,
    # but "no glove frames" warnings would otherwise clutter the first screen.
    nodes = []
    for hand in hands:
        cmd = [py, "-m", "v5s_teleop.ros2.teleop_node", "--hand", hand, "--num", NUM[hand]]
        if flag("dry_run"):
            cmd.append("--dry-run")
        if flag("diag"):
            cmd.append("--diag")
        nodes.append(ExecuteProcess(cmd=cmd, name=f"teleop_{hand}",
                                    cwd=str(ROOT), output="screen"))
    for hand in haptics:
        cmd = [py, "-m", "v5s_teleop.ros2.haptics_node", "--hand", hand, "--num", NUM[hand]]
        if flag("dry_run"):
            cmd.append("--dry-run")
        nodes.append(ExecuteProcess(cmd=cmd, name=f"haptics_{hand}",
                                    cwd=str(ROOT), output="screen"))
    if nodes:
        actions.append(TimerAction(period=float(arg("startup_delay")), actions=nodes))

    # If the bridge dies, take everything down.
    if bridge_action is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=bridge_action,
            on_exit=[LogInfo(msg="[v5s] the bridge exited -- shutting everything down"),
                     Shutdown(reason="manus_bridge exited")])))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("hands", default_value="left",
                              choices=["left", "right", "both"],
                              description="which hand gets teleop and a driver"),
        DeclareLaunchArgument("haptics", default_value="same",
                              choices=["same", "none", "left", "right", "both"],
                              description="which hand gets vibration (same = follow hands)"),
        DeclareLaunchArgument("driver", default_value="true",
                              description="bring up the hand drivers as well"),
        DeclareLaunchArgument("bridge", default_value="true",
                              description="bring up the glove bridge as well"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="do not actually emit joint_cmd or vibration"),
        DeclareLaunchArgument("diag", default_value="false",
                              description="turn on teleop latency instrumentation"),
        DeclareLaunchArgument("python", default_value="",
                              description="interpreter for the nodes; defaults to "
                                          "<repo>/.venv/bin/python, then python3 on PATH"),
        DeclareLaunchArgument("startup_delay", default_value="6.0",
                              description="seconds to wait for the bridge. Over wireless "
                                          "the gloves take about 4 s to attach "
                                          "(measured 2026-09-01)"),
        DeclareLaunchArgument("can_left", default_value=CAN["left"]),
        DeclareLaunchArgument("can_right", default_value=CAN["right"]),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
