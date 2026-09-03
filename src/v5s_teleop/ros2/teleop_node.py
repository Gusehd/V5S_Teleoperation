"""Glove to retargeting to the ROS2 hand driver.

Receives frames from the glove bridge, solves joint angles with DexPilot and
publishes them on `allegroHand_<NUM>/joint_cmd`.

    # log values only, publish nothing (check before touching hardware)
    python -m v5s_teleop.ros2.teleop_node --dry-run

    # publish for real
    python -m v5s_teleop.ros2.teleop_node --num 0

`rclpy` comes from the ROS2 distribution. Source it before running:

    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

import v5s_teleop
from v5s_teleop.bridge.zmq_client import GloveSubscriber
from v5s_teleop.retarget.builder import RetargetingSpec, build_retargeting

# The bridge publishes each hand on its own port. Putting both on one CONFLATE
# socket would let the two hands' frames overwrite each other -- see the header
# comment in bridge_cpp/manus_bridge.cpp.
GLOVE_ENDPOINTS = {"left": "tcp://127.0.0.1:5555", "right": "tcp://127.0.0.1:5557"}

# The order the driver expects; must match jointNames in allegro_node.cpp.
# The driver does `desired_position[i] = msg->position[i]` -- it reads **only
# the index** and never the name field. The order is the contract.
DRIVER_JOINT_ORDER: tuple[str, ...] = tuple(f"joint_{i}_0" for i in range(16))


class TeleopNode(Node):
    def __init__(self, args):
        # Running both hands at once would collide on the node name, and then
        # `ros2 param set /v5s_teleop ...` becomes ambiguous, which blocks tuning.
        super().__init__(f"v5s_teleop_{args.hand}")

        cfg = Path(args.config) if args.config else (
            Path(v5s_teleop.__file__).parent / f"configs/v5s_{args.hand}_dexpilot.yml")
        spec = RetargetingSpec.from_yaml(cfg)
        if args.scaling is not None:
            spec.scaling_factor = args.scaling
        if args.collision_weight is not None:
            spec.collision_weight = args.collision_weight
        if args.shape_weight is not None:
            spec.shape_weight = args.shape_weight
        if args.wrist_offset is not None:
            o = args.wrist_offset
            if len(o) == 1:
                spec.wrist_offset = [0.0, 0.0, float(o[0])]
            elif len(o) == 3:
                spec.wrist_offset = [float(v) for v in o]
            else:
                raise ValueError(f"--wrist-offset takes 1 value (z) or 3 (x y z): {o}")

        self.get_logger().info(f"config {cfg.name} / URDF {Path(spec.urdf_path).name}")
        self.get_logger().info(
            f"scaling {spec.scaling_factor}  eta1 {spec.eta1 * 1000:.0f}mm  "
            f"wrist {spec.wrist_link_name} offset {spec.wrist_offset}  "
            f"self-collision {'ON w=' + str(spec.collision_weight) if spec.collision_weight > 0 else 'off'}  "
            f"shape-match {'ON w=' + str(spec.shape_weight) if spec.shape_weight > 0 else 'off'}")

        self.retargeting = build_retargeting(spec)
        self.optimizer = self.retargeting.optimizer
        self.ref_origin, self.ref_task = self.optimizer.target_link_human_indices

        # retarget() returns values in robot.dof_joint_names (pinocchio) order.
        # The driver expects joint_0_0..joint_15_0 in sequence. These differ, so
        # reorder **by name** and never rely on index order.
        # Skip this and the thumb commands go to the middle finger.
        solver_order = self._bind_driver_order()
        self.get_logger().info(
            f"joint reorder  solver {solver_order[:4]}... -> driver {list(DRIVER_JOINT_ORDER[:4])}...")

        # Exposed as ROS parameters so they can be tuned while hardware runs.
        #   ros2 param set /v5s_teleop scaling_factor 0.7
        #   ros2 param set /v5s_teleop low_pass_alpha 0.1
        self.spec = spec
        self.declare_parameter("scaling_factor", float(spec.scaling_factor))
        self.declare_parameter("low_pass_alpha", float(spec.low_pass_alpha))
        # The offset changes the URDF structure (where the virtual wrist link
        # sits), so the retargeter has to be rebuilt. That takes about a second,
        # which still beats restarting when four dimensions have to be swept by
        # hand. Shape matching strength applies immediately with no rebuild --
        # the optimizer reads it every frame.
        #   ros2 param set /v5s_teleop shape_weight 0.05
        self.declare_parameter("shape_weight", float(spec.shape_weight))
        self.declare_parameter("wrist_offset", [float(v) for v in spec.wrist_offset])
        self.add_on_set_parameters_callback(self._on_params)

        self.dry_run = args.dry_run
        topic = f"allegroHand_{args.num}/joint_cmd"
        if self.dry_run:
            self.publisher = None
            self.get_logger().warn(f"DRY RUN -- not publishing to {topic}")
        else:
            # ── Publisher QoS ────────────────────────────────────────────
            # Measured 2026-08-25: with depth=1 and RELIABLE, publish() blocked
            # for 54-633 ms (0.1 ms is normal). The whole callback stalls with
            # it and the hand stutters.
            #
            # BEST_EFFORT would be the semantically right choice for teleop, but
            # **it cannot be used.** The driver (allegro_node_grasp_0) subscribes
            # as RELIABLE, so the QoS would be incompatible and the connection
            # would simply not form (confirmed with ros2 topic info -v). That
            # same node subscribes to joint_cmd twice.
            #
            # depth uses the ROS2 default of 10. We previously used 1, but paired
            # with RELIABLE it does not behave as "keep only the newest command"
            # -- the publisher ends up tied to subscriber ACKs. With 10 there is
            # 83 ms of slack (at 120 Hz) if the driver pauses briefly. **This is
            # not a fix for the stutter** -- that cause is still unconfirmed and
            # correlates more strongly with CPU contention from other work.
            #
            # lifespan is off by default. It is a writer-side-only policy that
            # lets DDS drop stale commands, so it does not affect subscriber
            # compatibility -- but it discards data silently, so it stays off
            # until verified.
            qos = QoSProfile(depth=args.qos_depth,
                             reliability=ReliabilityPolicy.RELIABLE)
            if args.qos_lifespan_ms > 0:
                qos.lifespan = Duration(nanoseconds=int(args.qos_lifespan_ms * 1e6))
            self.publisher = self.create_publisher(JointState, topic, qos)
            self.get_logger().info(
                f"publishing {topic}  QoS=RELIABLE depth={args.qos_depth} "
                f"lifespan={'infinite' if args.qos_lifespan_ms <= 0 else f'{args.qos_lifespan_ms}ms'}")

        # Watch the driver's own output to see whether it is alive. The point is
        # to tell whether the moments our publish blocks line up with gaps in the
        # driver's /joint_states. Subscribe BEST_EFFORT -- it is compatible with
        # the driver's RELIABLE publisher (requested weaker than offered is fine)
        # and it does not introduce backpressure of our own.
        if args.diag:
            self.create_subscription(
                JointState, f"allegroHand_{args.num}/joint_states", self._on_states,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.get_logger().info(f"diagnostics: watching allegroHand_{args.num}/joint_states")

        endpoint = args.endpoint or GLOVE_ENDPOINTS[args.hand]
        self.get_logger().info(f"glove stream {endpoint}")
        self.glove = GloveSubscriber(endpoint)
        self.frames = 0
        self.stale = 0

        # ── Stutter diagnostics ───────────────────────────────────────────
        # The cause only shows up when the layers are measured separately. Each
        # frame carries the sender's clock (t_ns), which lets us tell "the glove
        # did not send" apart from "we did not receive".
        #
        #   send_gap  gap in sender t_ns   -> did the glove or SDK stall?
        #   recv_gap  gap in receive time  -> did transport or the receiver stall?
        #   solve     retargeting solve time
        #   pub       publish() time       -> hit ROS2 backpressure? (RELIABLE QoS)
        #   tick      gap between callbacks -> did our own process freeze?
        #
        # Because of CONFLATE, a stall on our side also inflates the send gap.
        # If tick is normal (about 1 ms) and only send/recv are large, **the data
        # did not arrive**. If tick is large too, **our process stalled**.
        self.diag = args.diag
        self._t_prev_recv = None
        self._t_prev_send = None
        self._t_prev_tick = None
        self._worst = {"send": 0.0, "recv": 0.0, "solve": 0.0, "pub": 0.0,
                       "tick": 0.0, "drv": 0.0}
        self._t_prev_drv = None
        self._worst_at = 0.0
        # Process glove frames as they arrive. The driver's control loop runs at
        # 333 Hz, so forwarding the glove's 120 Hz does not back it up.
        self.timer = self.create_timer(0.001, self.tick)
        self.log_timer = self.create_timer(1.0, self.report)

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "scaling_factor":
                if p.value <= 0:
                    return SetParametersResult(successful=False, reason="scaling_factor must be positive")
                self.optimizer.scaling = float(p.value)
                self.get_logger().info(f"scaling_factor -> {p.value}")
            elif p.name == "low_pass_alpha":
                if not 0.0 < p.value <= 1.0:
                    return SetParametersResult(successful=False, reason="low_pass_alpha must be in (0, 1]")
                if self.retargeting.filter is not None:
                    self.retargeting.filter.alpha = float(p.value)
                self.get_logger().info(f"low_pass_alpha -> {p.value}")
            elif p.name == "shape_weight":
                if p.value < 0:
                    return SetParametersResult(successful=False, reason="shape_weight must be at least 0")
                if p.value > 0 and not self.optimizer._shape_configured:
                    return SetParametersResult(
                        successful=False, reason="the config has no shape_segments")
                self.optimizer.shape_weight = float(p.value)
                self.spec.shape_weight = float(p.value)
                self.get_logger().info(f"shape_weight -> {p.value}")
            elif p.name == "wrist_offset":
                if len(p.value) != 3:
                    return SetParametersResult(successful=False, reason="wrist_offset must be [x, y, z]")
                try:
                    self._rebuild(list(map(float, p.value)))
                except Exception as e:  # noqa: BLE001 -- a rebuild failure must reject
                                    # the parameter, never kill the node
                    return SetParametersResult(successful=False, reason=f"rebuild failed: {e}")
                self.get_logger().info(f"wrist_offset -> {list(p.value)} (retargeter rebuilt)")
        return SetParametersResult(successful=True)

    def _rebuild(self, offset: list[float]) -> None:
        """Rebuild the retargeter with a new offset, keeping scaling and the filter
        coefficient."""
        scaling = self.optimizer.scaling
        alpha = self.retargeting.filter.alpha if self.retargeting.filter else None
        self.spec.wrist_offset = offset
        self.spec.scaling_factor = scaling
        self.spec.shape_weight = self.optimizer.shape_weight
        retargeting = build_retargeting(self.spec)
        if alpha is not None and retargeting.filter is not None:
            retargeting.filter.alpha = alpha
        self.retargeting = retargeting
        self.optimizer = retargeting.optimizer
        self.ref_origin, self.ref_task = self.optimizer.target_link_human_indices
        self._bind_driver_order()

    def _bind_driver_order(self) -> list[str]:
        """Build the solver-order to driver-order reorder table and the limits.

        Called from both `__init__` and the parameter rebuild path. The same
        computation used to be copied into two places; fixing only one of them
        would scramble the commands the moment `wrist_offset` changed.

        Returns:
            Joint names in solver (pinocchio) order, for logging.
        """
        solver_order = list(self.optimizer.robot.dof_joint_names)
        missing = set(DRIVER_JOINT_ORDER) - set(solver_order)
        if missing:
            raise RuntimeError(
                f"the driver expects joints that are not in the URDF: {sorted(missing)}")
        self.to_driver = np.array([solver_order.index(j) for j in DRIVER_JOINT_ORDER])

        # Limits for the hard clamp applied just before publishing, kept in
        # **driver order**. joint_limits is in target order, so reorder it by
        # name -- relying on index order would fall into the same trap as
        # to_driver above.
        target_names = list(self.optimizer.target_joint_names)
        idx = [target_names.index(j) for j in DRIVER_JOINT_ORDER]
        limits = np.asarray(self.retargeting.joint_limits, dtype=np.float64)[idx]
        self.driver_lo, self.driver_hi = limits[:, 0], limits[:, 1]
        return solver_order

    def _on_states(self, _msg) -> None:
        """Gap between driver outputs. This widens when the driver stalls."""
        now = time.perf_counter()
        if self._t_prev_drv is not None:
            self._mark("drv", (now - self._t_prev_drv) * 1e3)
        self._t_prev_drv = now

    def _mark(self, key: str, ms: float) -> None:
        """Keep only this period's maximum (hot path, so no allocation)."""
        if ms > self._worst[key]:
            self._worst[key] = ms
            self._worst_at = max(self._worst_at, ms)

    def tick(self) -> None:
        if self.diag:
            t_in = time.perf_counter()
            if self._t_prev_tick is not None:
                self._mark("tick", (t_in - self._t_prev_tick) * 1e3)
            self._t_prev_tick = t_in

        frame = self.glove.recv(timeout_ms=0)
        if frame is None:
            self.stale += 1
            return

        if self.diag:
            now = time.perf_counter()
            if self._t_prev_recv is not None:
                self._mark("recv", (now - self._t_prev_recv) * 1e3)
            self._t_prev_recv = now
            if self._t_prev_send is not None:
                # t_ns is the sender's monotonic clock. Only differences are used,
                # so the two clocks need not agree.
                self._mark("send", (frame.t_ns - self._t_prev_send) / 1e6)
            self._t_prev_send = frame.t_ns
            t0 = now

        mano = frame.to_mano21()
        # Shape matching needs the intermediate joint positions, but upstream's
        # retarget() only takes ten vectors. So they are supplied separately each
        # frame (ignored when shape matching is off).
        self.optimizer.set_human_keypoints(mano)
        ref = mano[self.ref_task] - mano[self.ref_origin]
        qpos = self.retargeting.retarget(ref)   # pinocchio order
        self.frames += 1
        self.last_qpos = qpos
        if self.diag:
            self._mark("solve", (time.perf_counter() - t0) * 1e3)

        if self.publisher is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(DRIVER_JOINT_ORDER)
        # Hardware safety rule 1: **final clamp** to the URDF limits. Do not
        # trust the optimizer -- SLSQP can return a value about 1e-3 rad outside
        # the bound (measured 0.057 degrees). Harmless at that size, but the
        # defence must not rest on numerical slop.
        cmd = np.clip(qpos[self.to_driver], self.driver_lo, self.driver_hi)
        msg.position = [float(v) for v in cmd]
        if self.diag:
            t1 = time.perf_counter()
            self.publisher.publish(msg)
            self._mark("pub", (time.perf_counter() - t1) * 1e3)
        else:
            self.publisher.publish(msg)

    def report(self) -> None:
        if self.frames == 0:
            self.get_logger().warn("no glove frames -- is manus_bridge running?")
        else:
            q = np.degrees(self.last_qpos[self.to_driver])
            self.get_logger().info(
                f"{self.frames} Hz  s={self.optimizer.scaling:.2f} "
                f"o={[round(v, 3) for v in self.spec.wrist_offset]} "
                f"w={self.optimizer.shape_weight:g} | "
                f"index {q[0]:+6.1f}{q[1]:+6.1f}{q[2]:+6.1f}{q[3]:+6.1f} | "
                f"middle {q[4]:+6.1f}{q[5]:+6.1f}{q[6]:+6.1f}{q[7]:+6.1f} | "
                f"thumb {q[12]:+6.1f}{q[13]:+6.1f}{q[14]:+6.1f}{q[15]:+6.1f}")
        if self.diag and self._worst_at > 0.0:
            w = self._worst
            # 8.3 ms is one frame at 120 Hz. Whichever term far exceeds it is
            # the culprit.
            worst_key = max(w, key=w.get)
            self.get_logger().info(
                f"  diag max[ms]  glove-send {w['send']:7.1f} | recv {w['recv']:7.1f} | "
                f"tick {w['tick']:6.1f} | solve {w['solve']:5.1f} | publish {w['pub']:6.1f} | "
                f"driver {w['drv']:6.1f}   <- {worst_key}")
            self._worst = {k: 0.0 for k in w}
            self._worst_at = 0.0
        self.frames = 0
        self.stale = 0

    def destroy_node(self) -> None:
        self.glove.close()
        super().destroy_node()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", default="left", choices=("left", "right"))
    ap.add_argument("--num", default="0", help="the NUM in allegroHand_<NUM>")
    ap.add_argument("--config", default=None)
    ap.add_argument("--scaling", type=float, default=None)
    ap.add_argument("--wrist-offset", type=float, nargs="+", default=None,
                    metavar="V",
                    help="virtual wrist offset [m]. One value sets z only "
                         "(e.g. -0.189), three set x y z (e.g. 0.02 0 -0.189). "
                         "Overrides the config file.")
    ap.add_argument("--endpoint", default=None,
                    help=f"glove stream address; defaults to the one for --hand "
                         f"({GLOVE_ENDPOINTS['left']} / {GLOVE_ENDPOINTS['right']})")
    ap.add_argument("--collision-weight", type=float, default=None,
                    help="self-collision penalty strength. 0 disables it (the "
                         "config default). Turn it on only when fingers are visibly "
                         "hitting each other. Suggested starting value 1000")
    ap.add_argument("--shape-weight", type=float, default=None,
                    help="shape matching strength. 0 disables it (the config "
                         "default). Makes the finger segments bend at the same "
                         "angles as the human hand. Useful range 0.01-0.2 -- larger "
                         "resembles the hand shape more but loses fingertip "
                         "accuracy. Start at 0.05")
    ap.add_argument("--qos-depth", type=int, default=10,
                    help="publisher queue depth (ROS2 default 10). Pass 1 to "
                         "reproduce the previous behaviour, for A/B testing the "
                         "stutter")
    ap.add_argument("--qos-lifespan-ms", type=float, default=0.0,
                    help="let DDS discard commands older than this. 0 disables it "
                         "(the default). Unverified, so it stays off by default")
    ap.add_argument("--diag", action="store_true",
                    help="stutter diagnostics: print the per-layer maximum delay "
                         "every second (glove send / recv / solve / publish)")
    ap.add_argument("--dry-run", action="store_true", help="do not publish, only log the values")
    args = ap.parse_args()

    rclpy.init()
    node = TeleopNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # After a Ctrl+C, rclpy may already have torn the context down. Calling
        # shutdown anyway raises RCLError and makes the exit messy.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
