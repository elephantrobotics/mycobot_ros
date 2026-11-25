#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Validated Pro630 slider-command bridge for Gazebo and optional real hardware."""

import math
import queue
import signal
import threading
import time

import pro630_ros1 as ros
from rospy import Duration, Time
from moveit_msgs.srv import GetStateValidity
from pro630_ros1 import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String, Bool
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pro630_real_keyboard import RealPoseReader

from pro630_public_client import PublicPro630Client as Pro630Client
from pro630_feedback import attach_feedback
from pro630_control_logging import emit_log
from pro630_gripper_control import gripper_speed_for_percentage
from pro630_scene_support import CollisionSceneSupport
from pro630_collision_check import (
    CollisionCheck, COLLISION_SAMPLE_STEP, MAX_COLLISION_SAMPLES,
    DEFAULT_COLLISION_DEPTH_TOLERANCE_M, SHOW_COLLISION_PATH_PERCENT,
    SHOW_COLLISION_DEPTH, SHOW_COLLISION_BODIES,
)


from pro630_model import (DEFAULT_PRO630_IP, DEFAULT_PRO630_PORT,
                         ARM_JOINT_SDK_SPEED_MAX, ARM_FULL_SPEED_RAD_S)

from pro630_configuration import ARM_JOINTS, GRIPPER_JOINT, JOINT_LIMITS_RAD, joints, configure_node
COMMAND_JOINTS = joints(True)
# Match Pro450 slider timing in both modes; these are feature speed limits.
MAX_VELOCITY_RAD_S = (1.0,) * 7
SIMULATION_MAX_VELOCITY_RAD_S = (1.0,) * 7
MIN_TRAJECTORY_DURATION = 0.5
STOP_HOLD_DURATION = 0.05
MAX_TRAJECTORY_DURATION = 600.0
SPLINE_PEAK_VELOCITY_FACTOR = 1.5
SIMULATION_ARRIVAL_TOLERANCE = 0.01
REAL_POSE_TOLERANCE_RAD = 0.05
WAYPOINT_ARM_TOLERANCE_RAD = math.radians(0.5)
WAYPOINT_GRIPPER_TOLERANCE = 0.05
# Distance from the MoveIt-checked joint-space line that stops the real arm.
LINE_DEVIATION_LIMIT_RAD = math.radians(1.0)
REAL_ARRIVAL_MARGIN_S = 10.0


def slider_sdk_speed_for_rad(velocity):
    """Match Pro450's 1.5 deg/s speed steps using Pro630's native range.

    Full scale is 90 deg/s. Preserve Pro450 slider quantization and round
    down when mapping the resulting percentage to the Pro630 command.
    """
    if not math.isfinite(velocity) or velocity < 0:
        raise ValueError('Invalid slider angular speed')
    # Keep the 1.5 deg/s feature steps independent of native full-scale calibration.
    step = math.radians(1.5)
    steps = max(1, int((min(velocity, 1.0) + 1e-12) / step))
    return max(1, int(steps * step / ARM_FULL_SPEED_RAD_S * ARM_JOINT_SDK_SPEED_MAX))


def sdk_motion_accepted(result):
    """ElephantRobot write_angles returns None; measured arrival is authoritative."""
    return result is None or (type(result) is int and result in (0, 1))


def gripper_write_accepted(result):
    """Pro630 force commands return an integer; never mask -1 as success."""
    return type(result) is int and result == 1


class SliderControl(CollisionCheck, Node):
    _wait_for_future = CollisionSceneSupport._wait_for_future

    def __init__(self):
        super().__init__("slider_control_gazebo")
        configure_node(self)

        self.declare_parameter("mode", "simulation")
        self.declare_parameter("pro630_ip", DEFAULT_PRO630_IP)
        self.declare_parameter("pro630_port", DEFAULT_PRO630_PORT)
        self.declare_parameter("feedback_hz", 10.0)
        self.declare_parameter("startup_stable_samples", 5)
        self.declare_parameter("startup_sample_interval_sec", 0.2)
        self.declare_parameter("startup_stable_tolerance_deg", 0.2)
        # Keep the MoveIt ground surface coincident with Gazebo's z=0 plane.
        # A positive value is an optional safety margin, not model geometry.
        self.declare_parameter(
            "collision_depth_tolerance_m",
            DEFAULT_COLLISION_DEPTH_TOLERANCE_M,
        )
        mode_name = str(self.get_parameter("mode").value).strip().lower()
        if mode_name not in ("simulation", "real"):
            raise ValueError("mode must be either 'simulation' or 'real'.")
        self.mode = 2 if mode_name == "real" else 1
        self.startup_stable_samples = max(
            2, int(self.get_parameter("startup_stable_samples").value)
        )
        self.startup_sample_interval_sec = max(
            0.05,
            float(self.get_parameter("startup_sample_interval_sec").value),
        )
        self.startup_stable_tolerance_deg = max(
            0.0,
            float(self.get_parameter("startup_stable_tolerance_deg").value),
        )
        self.pro630_ip = self.get_parameter("pro630_ip").value
        self.pro630_port = int(self.get_parameter("pro630_port").value)
        self.collision_depth_tolerance_m = max(
            0.0,
            float(self.get_parameter("collision_depth_tolerance_m").value),
        )
        self.pub_arm = self.create_publisher(
            JointTrajectory, "/arm_controller/command", 10
        )
        self.pub_gripper = self.create_publisher(
            JointTrajectory, "/pro_gripper_controller/command", 10
        ) if self.with_gripper else None
        self.status_pub = self.create_publisher(String, "/pro630/slider_status", 10)

        self.create_subscription(JointState, "/joint_states", self._feedback_cb, 10)
        self.create_subscription(
            JointState, "/pro630/slider_targets", self._target_cb, 10
        )
        self.create_subscription(
            JointState, "/pro630/slider_force_targets", self._force_target_cb, 10
        )
        self.create_subscription(Empty, "/pro630/slider_stop", self._stop_cb, 10)

        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity", persistent=True
        )
        self.get_limited_client = (
            self.create_client(Trigger, "/pro630/get_limited_tracking")
            if self.mode == 1 else None
        )
        self.set_limited_client = (
            self.create_client(SetBool, "/pro630/set_limited_tracking")
            if self.mode == 1 else None
        )
        self.set_hold_client = (
            self.create_client(SetBool, "/pro630/set_limited_hold")
            if self.mode == 1 else None
        )
        self._limited_tracking = False
        self._hold_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._current_positions = None
        self._feedback_time = 0.0
        self._feedback_valid = False
        self._command_active = False
        self._stop_requested = False
        self._stop_event = threading.Event()
        # The read-only startup snapshot must not advertise idle while the
        # active controller is validating/executing a command.
        self.busy_pub = self.create_publisher(Bool, "/pro630/slider_busy", 10)
        self.create_timer(0.1, self._publish_command_busy)
        self.mc = None
        self._real_thread = None
        self._motion_sent = False
        self._pose_reader = None
        self._arm_moving = False
        self._last_read_error = 0.0
        self._real_motion_enabled = self.mode == 2
        self.command_queue = queue.Queue(maxsize=1)
        if self.mode == 2:
            self._initialize_pro630()
            self._pose_reader = RealPoseReader(
                self.mc, self.joint_limits, report=self.get_logger().debug, with_gripper=self.with_gripper)
            if self.with_gripper:
                self._pose_reader.seed_gripper(self._startup_pose[6])
            stamp = self.mc.sample_time('arm')
            if stamp is not None:
                self._pose_reader.observe_arm(
                    [math.degrees(v) for v in self._startup_pose[:6]], *stamp)
            attach_feedback(self, self._pose_reader)
            self._real_thread = threading.Thread(target=self._real_robot_worker, daemon=True)
            self._real_thread.start()
            self._publish_status("Ready: Real Robot + Gazebo motion mode.")
        else:
            self._publish_status("Ready: Gazebo simulation mode.")

    def _publish_status(self, message, level="info"):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self._publish_command_busy()
        emit_log(self.get_logger(), message, level)

    def _publish_command_busy(self):
        if self.busy_pub is not None:
            msg = Bool()
            # Also called while _state_lock is held on duplicate rejection.
            with self._state_lock:
                msg.data = self._command_active
                self.busy_pub.publish(msg)


    def _read_startup_gripper(self, timeout=10.0):
        """Acquire a real initial opening, with a deadline even for a blocked SDK read."""
        sdk = self.mc
        finished = threading.Event()
        cancelled = threading.Event()
        result = {'attempts': 0, 'last': None}
        deadline = time.monotonic() + timeout

        def read_until_valid():
            try:
                while not cancelled.is_set() and time.monotonic() < deadline:
                    result['attempts'] += 1
                    try:
                        value = sdk.get_pro_gripper_angle()
                    except Exception as exc:
                        value = f"{type(exc).__name__}: {exc}"
                    result['last'] = value
                    if cancelled.is_set() or time.monotonic() >= deadline:
                        return
                    if (not isinstance(value, bool) and isinstance(value, (int, float))
                            and math.isfinite(value) and 0.0 <= value <= 100.0):
                        result['value'] = float(value)
                        return
                    cancelled.wait(min(0.1, max(0.0, deadline - time.monotonic())))
            finally:
                finished.set()

        self.get_logger().info(f"Reading initial gripper angle; timeout {timeout:.1f}s...")
        worker = threading.Thread(target=read_until_valid, daemon=True)
        worker.start()
        finished.wait(max(0.0, deadline - time.monotonic()))
        cancelled.set()
        if 'value' not in result:
            # Close the read-only connection to unblock a pending SDK query.
            # No pose, speed or motion commands are issued during this retry.
            sdk.close()
            worker.join(timeout=1.0)
            raise RuntimeError(
                f"initial gripper angle read timed out after {timeout:.1f}s "
                f"({result['attempts']} attempts, last={result['last']!r})")
        worker.join()
        self.get_logger().info(
            f"Initial gripper angle acquired: {result['value']:.1f} "
            f"after {result['attempts']} read(s).")
        return result['value']

    def _initialize_pro630(self):
        if Pro630Client is None:
            raise RuntimeError("pymycobot is required only for real Pro630 mode")
        try:
            self.get_logger().info(
                f"Connecting to Pro630 @ {self.pro630_ip}:{self.pro630_port}"
            )
            self.mc = Pro630Client(self.pro630_ip, self.pro630_port,
                                   feedback_hz=float(self.get_parameter("feedback_hz").value), feature="slider", with_gripper=self.with_gripper, allow_collision=self.allow_collision)
            power_state = self.mc.is_power_on()
            if power_state != 1:
                raise RuntimeError(
                    f"robot is not powered and will not be powered automatically "
                    f"(is_power_on={power_state})"
                )

            error_code = self.mc.get_error_information()
            if error_code not in (0, None):
                raise RuntimeError(f"robot reports error code {error_code}")

            samples = []
            for index in range(self.startup_stable_samples):
                moving = self.mc.is_moving()
                if moving != 0:
                    raise RuntimeError(
                        f"robot must be stationary for pose synchronization "
                        f"(is_moving={moving})"
                    )
                angles = self.mc.get_angles()
                samples.append(self._validate_real_angles(angles))
                if index + 1 < self.startup_stable_samples:
                    time.sleep(self.startup_sample_interval_sec)

            for joint_index in range(len(ARM_JOINTS)):
                values = [sample[joint_index] for sample in samples]
                if max(values) - min(values) > self.startup_stable_tolerance_deg:
                    raise RuntimeError(
                        f"{ARM_JOINTS[joint_index]} did not remain stable during "
                        "startup sampling"
                    )

            gripper_value = self._read_startup_gripper(timeout=10.0) if self.with_gripper else None

            averaged_angles = [
                sum(sample[index] for sample in samples) / len(samples)
                for index in range(len(ARM_JOINTS))
            ]
            self._startup_pose = [math.radians(value) for value in averaged_angles] + ([gripper_value / 100.0] if self.with_gripper else [])
            self.get_logger().info(
                "Pro630 connected in read-only startup phase. Stable angles: "
                f"{averaged_angles}." + (f" Gripper={gripper_value:.1f}." if self.with_gripper else "")
            )
        except Exception as exc:
            if self.mc is not None:
                self.mc.close()
            self.mc = None
            raise RuntimeError(f"Unable to initialize Pro630: {exc}") from exc


    @staticmethod
    def _validate_real_angles(angles):
        if not isinstance(angles, (list, tuple)) or len(angles) != len(ARM_JOINTS):
            raise RuntimeError(f"invalid joint angle response: {angles!r}")
        result = []
        for name, raw_value, limits in zip(ARM_JOINTS, angles, JOINT_LIMITS_RAD):
            if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
                raise RuntimeError(f"invalid {name} angle: {raw_value!r}")
            value = float(raw_value)
            if not math.isfinite(value):
                raise RuntimeError(f"invalid {name} angle: {raw_value!r}")
            lower_deg = math.degrees(limits[0])
            upper_deg = math.degrees(limits[1])
            if not lower_deg <= value <= upper_deg:
                raise RuntimeError(
                    f"{name}={value:.4f} deg is outside [{lower_deg}, {upper_deg}]"
                )
            result.append(value)
        return result

    def _feedback_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in self.command_joints):
            return
        positions = [float(values[name]) for name in self.command_joints]
        velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        feedback_valid = all(math.isfinite(value) for value in positions) and all(
            name in velocities and math.isfinite(float(velocities[name]))
            for name in self.command_joints
        )
        with self._state_lock:
            self._current_positions = positions
            self._feedback_time = time.monotonic()
            self._feedback_valid = feedback_valid

    def _target_cb(self, msg):
        if self.mode == 2 and not self._real_motion_enabled:
            return
        self._handle_target(msg, force_collision=False)

    def _force_target_cb(self, msg):
        if self.mode != 1:
            self._publish_status(
                "Rejected: collision override is disabled in Real Robot + Gazebo mode.", level="warning"
            )
            return
        self._handle_target(msg, force_collision=True)

    def _handle_target(self, msg, force_collision):
        if "teleop_keyboard_gazebo" in self.get_node_names():
            self._publish_status("Rejected: keyboard controller is running.", level="warning")
            return
        values = dict(zip(msg.name, msg.position))
        if (len(msg.name) != len(self.command_joints) or len(msg.position) != len(self.command_joints)
                or set(msg.name) != set(self.command_joints)):
            self._publish_status("Rejected: target joints do not match the installation profile.", level="warning")
            return

        target = [float(values[name]) for name in self.command_joints]
        if not all(math.isfinite(value) for value in target):
            self._publish_status("Rejected: target contains NaN or infinity.", level="warning")
            return

        for name, value, limits in zip(self.command_joints, target, self.joint_limits):
            if not limits[0] <= value <= limits[1]:
                self._publish_status(
                    f"Rejected: {name}={value:.4f} rad is outside its limits.", level="warning"
                )
                return

        # Validate and execute the same discrete opening sent to the SDK.
        if self.with_gripper:
            target[-1] = round(target[-1] * 100) / 100.0

        speed_scale = 0.2
        if msg.velocity:
            finite_speeds = [abs(value) for value in msg.velocity if math.isfinite(value)]
            if finite_speeds:
                speed_scale = max(finite_speeds)
        speed_scale = max(0.01, min(1.0, speed_scale))

        if self.mode == 2:
            self._queue_real_motion(target, speed_scale)
            return

        with self._state_lock:
            if self._command_active:
                self._publish_status("Rejected: a command is already being validated/executed.", level="warning")
                return
            current = (
                list(self._current_positions)
                if self._current_positions is not None
                else None
            )
            feedback_age = time.monotonic() - self._feedback_time
            feedback_valid = self._feedback_valid
            self._command_active = True
            self._stop_requested = False
            self._stop_event.clear()

        self._publish_command_busy()
        threading.Thread(
            target=self._validate_and_execute,
            args=(current, target, speed_scale, force_collision),
            daemon=True,
        ).start()

    def _validate_and_execute(self, current, target, speed_scale, force_collision):
        try:
            # The GUI and backend discover /joint_states independently. A
            # first click can precede the backend's first sample by a few ms.
            # Wait off the ROS callback thread so feedback can arrive; never
            # execute from a missing, invalid or stale pose.
            deadline = time.monotonic() + 1.0
            while True:
                if self._stop_requested:
                    self._publish_status("Cancelled before execution.")
                    return
                with self._state_lock:
                    current = (None if self._current_positions is None else
                               list(self._current_positions))
                    usable = (current is not None and self._feedback_valid and
                              time.monotonic() - self._feedback_time <= 1.0)
                if usable:
                    break
                if time.monotonic() >= deadline:
                    self._publish_status(
                        "Rejected: Gazebo position/velocity feedback is missing, stale, or contains NaN. Restart/reset simulation.", level="warning")
                    return
                self._stop_event.wait(.01)
            if force_collision:
                self._publish_status(
                    "WARNING: simulation-only collision override active; "
                    "MoveIt collision validation is being skipped.", level="warning"
                )
            else:
                self._publish_status("Validating interpolated path for collisions...")
                valid, reason = self._path_is_valid(current, target)
                if not valid:
                    self._publish_status(f"Rejected: {reason}", level="warning")
                    return

            duration = self._trajectory_duration(
                current, target, speed_scale, velocity_limits=SIMULATION_MAX_VELOCITY_RAD_S)
            if self._stop_requested:
                self._publish_status("Cancelled before execution.")
                return

            if force_collision:
                self._set_limited_tracking(True)
            elif self._get_limited_tracking():
                with self._state_lock:
                    align = (list(self._current_positions)
                             if self._current_positions is not None else current)
                self._publish_trajectory(align, 0.05)
                if self._stop_event.wait(0.07) or self._stop_requested:
                    self._publish_status("Cancelled before execution.")
                    return
                self._set_limited_tracking(False)
                # Stay on the measured pose until rigid tracking is quiescent,
                # then let the validated trajectory start from that pose.
                rigid_deadline = time.monotonic() + 15.0
                while self._get_limited_tracking():
                    if self._stop_event.wait(0.02) or self._stop_requested:
                        self._publish_status("Cancelled before execution.")
                        return
                    if time.monotonic() >= rigid_deadline:
                        raise RuntimeError("Gazebo did not return to rigid tracking.")
                if self._stop_event.wait(0.05) or self._stop_requested:
                    self._publish_status("Cancelled before execution.")
                    return
                with self._state_lock:
                    if self._feedback_valid and self._current_positions is not None:
                        current = list(self._current_positions)
                duration = self._trajectory_duration(
                    current, target, speed_scale, velocity_limits=SIMULATION_MAX_VELOCITY_RAD_S)

            if self._stop_requested:
                self._publish_status("Cancelled before execution.")
                return

            start = Time.now().to_sec()
            self._publish_trajectory(target, duration)
            if force_collision:
                # Let the new setpoint replace the STOP anchor before tracking it.
                # The lock keeps a STOP that arrives in this window as the final hold.
                self._stop_event.wait(0.03)
                with self._hold_lock:
                    if not self._stop_requested:
                        self._set_limited_hold(False)

            execution_kind = "FORCE SIMULATION" if force_collision else "Executing"
            self._publish_status(
                f"{execution_kind} at {speed_scale * 100:.0f}% speed; "
                f"planned duration {duration:.2f} s."
            )
            self._wait_for_simulation_arrival(target, start, duration, force_collision)
        except Exception as exc:
            self._publish_status(f"Execution failed: {exc}", level="error")
        finally:
            with self._state_lock:
                self._command_active = False
            self._publish_command_busy()


    def _get_limited_tracking(self):
        if self.get_limited_client is None:
            return False
        if not self.get_limited_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Gazebo tracking mode query service is unavailable.")
        future = self.get_limited_client.call_async(Trigger._request_class())
        response, reason = self._wait_for_future(
            future, 2.0, "Querying Gazebo tracking mode"
        )
        if response is None:
            raise RuntimeError(reason)
        return bool(response.success)

    def _set_limited_tracking(self, enabled):
        if self.set_limited_client is None:
            if enabled:
                raise RuntimeError("Limited tracking is allowed only in simulation.")
            return
        if not self.set_limited_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Gazebo tracking mode switch service is unavailable.")
        request = SetBool._request_class()
        request.data = bool(enabled)
        future = self.set_limited_client.call_async(request)
        response, reason = self._wait_for_future(
            future, 2.0, "Switching Gazebo tracking mode"
        )
        if response is None:
            raise RuntimeError(reason)
        if not response.success:
            raise RuntimeError(response.message or "Gazebo rejected tracking mode switch.")
        self._limited_tracking = bool(enabled)

    def _set_limited_hold(self, enabled):
        if self.set_hold_client is None:
            if enabled:
                raise RuntimeError("STOP hold is available only in simulation.")
            return
        if not self.set_hold_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Gazebo STOP hold service is unavailable.")
        request = SetBool._request_class()
        request.data = bool(enabled)
        future = self.set_hold_client.call_async(request)
        response, reason = self._wait_for_future(future, 2.0, "Holding the current Gazebo pose")
        if response is None:
            raise RuntimeError(reason)
        if not response.success:
            raise RuntimeError(response.message or "Gazebo rejected the STOP hold.")

    @staticmethod
    def _trajectory_duration(current, target, speed_scale, velocity_limits=None):
        limits = MAX_VELOCITY_RAD_S if velocity_limits is None else velocity_limits
        durations = []
        for start, end, maximum in zip(current, target, limits):
            # joint_trajectory_controller uses spline interpolation.  With
            # zero endpoint velocity, peak speed is about 1.5x the average,
            # so size the duration for the peak rather than the average.
            durations.append(
                SPLINE_PEAK_VELOCITY_FACTOR
                * abs(end - start)
                / max(0.01, maximum * speed_scale)
            )
        return max(MIN_TRAJECTORY_DURATION, min(MAX_TRAJECTORY_DURATION, max(durations)))

    def _wait_for_simulation_arrival(self, target, start, duration, keep_tracking):
        # Gazebo's clock can advance more slowly than wall time in a VM.
        # Require fresh joint feedback near the target as well.
        deadline = time.monotonic() + max(10.0, 4.0 * duration + 5.0)
        while ros.ok() and not self._stop_event.is_set():
            if Time.now().to_sec() >= start + duration:
                with self._state_lock:
                    reached = (self._feedback_valid and self._current_positions is not None
                               and time.monotonic() - self._feedback_time <= 1.0
                               and all(abs(a - b) <= SIMULATION_ARRIVAL_TOLERANCE
                                       for a, b in zip(self._current_positions, target)))
                if reached and not self._stop_event.is_set():
                    self._publish_status("Execution complete.")
                    return
            if deadline is not None and time.monotonic() >= deadline:
                if keep_tracking:
                    self._publish_status(
                        "Target not reached. Force Execute is still active; use STOP to end it.",
                        level="warning")
                    deadline = None
                    continue
                self._stop_cb(None)
                self._publish_status("Execution failed: Gazebo did not reach the target.", level="error")
                return
            self._stop_event.wait(0.02)
        self._publish_status("Execution stopped.")

    def _publish_trajectory(self, target, duration, velocities=None):
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1_000_000_000)

        arm = JointTrajectory()
        # Keep the header stamp at zero: trajectory controllers interpret this
        # as "start now".  This avoids mixing wall time from this node with
        # Gazebo simulation time used by controller_manager.
        arm.joint_names = list(ARM_JOINTS)
        arm_point = JointTrajectoryPoint()
        arm_point.positions = list(target[:6])
        if velocities is not None:
            arm_point.velocities = list(velocities[:6])
        arm_point.time_from_start = Duration(secs=seconds, nsecs=nanoseconds)
        arm.points = [arm_point]
        self.pub_arm.publish(arm)

        if self.with_gripper:
            self._publish_gripper_trajectory(target[6], duration, None if velocities is None else velocities[6])

    def _publish_gripper_trajectory(self, position, duration, velocity=None):
        if not self.with_gripper:
            return
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1_000_000_000)
        gripper = JointTrajectory()
        gripper.joint_names = [GRIPPER_JOINT]
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [position]
        if velocity is not None:
            gripper_point.velocities = [velocity]
        gripper_point.time_from_start = Duration(secs=seconds, nsecs=nanoseconds)
        gripper.points = [gripper_point]
        self.pub_gripper.publish(gripper)

    def _queue_real_motion(self, target, speed_scale):
        with self._state_lock:
            if self._command_active:
                self._publish_status("Rejected: a command is already being validated/executed.", level="warning")
                return
            self._command_active = True
            self._stop_requested = False
            self._stop_event.clear()
        self._publish_command_busy()
        try:
            self.command_queue.put_nowait((list(target), speed_scale))
        except queue.Full:
            with self._state_lock:
                self._command_active = False
            self._publish_status("Rejected: a command is already being validated/executed.", level="warning")

    def _read_real_positions(self, moving=None):
        pose = self._pose_reader(moving=self._arm_moving if moving is None else moving)
        return pose

    def _gazebo_matches(self, positions):
        """True when Gazebo is showing this measured pose within the tolerance."""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self._stop_requested:
                return False
            with self._state_lock:
                gazebo = (
                    list(self._current_positions)
                    if self._current_positions is not None else None
                )
                age = time.monotonic() - self._feedback_time
                valid = self._feedback_valid
            if (gazebo is not None and valid and age <= 1.0 and
                    max(abs(left - right) for left, right in zip(gazebo, positions))
                    <= REAL_POSE_TOLERANCE_RAD):
                return True
            self._read_real_positions(moving=True)
            self._stop_event.wait(0.1)
        return False

    def _stop_real_motion(self):
        """SDK stop on the owner thread, then show the measured pose."""
        if self.mc is not None and self._motion_sent:
            self.mc.stop()
            if self.with_gripper:
                # Arm STOP does not stop the independent gripper motor.
                try:
                    opening = self._pose_reader.read_fresh_gripper()
                    self._send_gripper(int(round(opening * 100.0)))
                except Exception as exc:
                    self.get_logger().error(f"Could not hold gripper at measured opening: {exc}")
        self._motion_sent = False
        try:
            self._read_real_positions()
        except Exception as exc:
            self.get_logger().error(f"Could not mirror the stopped pose: {exc}")


    def _send_gripper(self, opening):
        if not self.with_gripper:
            raise RuntimeError("Gripper is not installed")
        result = self.mc.set_pro_gripper_angle(opening)
        if not gripper_write_accepted(result):
            raise RuntimeError(f"gripper returned {result!r}")

    def _follow_real_line(self, current, target, timeout):
        """Watch one firmware move and stop it if it leaves the checked line.

        Returns True on arrival, False on STOP. Raises on deviation, bad
        feedback, or timeout; the caller stops the arm.
        """
        start_arm, goal_arm = current[:6], target[:6]
        delta = [goal - start for start, goal in zip(start_arm, goal_arm)]
        length_sq = sum(value * value for value in delta)
        arm_moves = length_sq > WAYPOINT_ARM_TOLERANCE_RAD ** 2
        grip_goal = target[6] if self.with_gripper else None
        deadline = time.monotonic() + timeout
        bad_reads = 0
        self._arm_moving = True
        try:
            while ros.ok():
                if self._stop_requested:
                    return False
                try:
                    arm = self._read_real_positions(moving=True)[:6]
                except Exception:
                    bad_reads += 1
                    if bad_reads >= 3:
                        raise
                    time.sleep(0.05)
                    continue
                bad_reads = 0
                if arm_moves:
                    progress = sum(
                        (value - start) * step
                        for value, start, step in zip(arm, start_arm, delta)) / length_sq
                    progress = max(0.0, min(1.0, progress))
                    deviation = max(
                        abs(value - (start + step * progress))
                        for value, start, step in zip(arm, start_arm, delta))
                    if deviation > LINE_DEVIATION_LIMIT_RAD:
                        raise RuntimeError(
                            f"left the checked path by {math.degrees(deviation):.2f} deg")
                else:
                    progress = 1.0
                arm_done = all(
                    abs(value - goal) <= WAYPOINT_ARM_TOLERANCE_RAD
                    for value, goal in zip(arm, goal_arm))
                if arm_done and (not arm_moves or self.mc.is_moving() == 0):
                    if not self.with_gripper:
                        return True
                    try:
                        self._pose_reader.read_fresh_gripper()
                        measured = self._read_real_positions(moving=False)
                    except Exception:
                        measured = None
                    if (measured is not None and
                            # Normalized integer openings can differ by a
                            # rounding epsilon at the inclusive tolerance edge.
                            abs(measured[6] - grip_goal) <= WAYPOINT_GRIPPER_TOLERANCE + 1e-12):
                        return True
                if time.monotonic() >= deadline:
                    raise RuntimeError("timed out waiting for the real robot to arrive")
        finally:
            self._arm_moving = False
        return False

    def _validate_real_motion(self, current, target):
        """ROS collision calls run separately; only this caller touches SDK.

        The owner continues six-axis polling with cached gripper values while
        the validation thread waits for MoveIt. No writes occur in that thread.
        """
        finished = threading.Event()
        result = {}

        def validate():
            try:
                self._publish_status("Validating interpolated path for collisions...")
                valid, reason = self._path_is_valid(current, target)
                result['value'] = valid, reason
            except Exception as exc:
                result['error'] = exc
            finally:
                finished.set()

        worker = threading.Thread(target=validate, daemon=True)
        worker.start()
        failure = None
        next_read = time.monotonic()
        while not finished.wait(0.02):
            if not ros.ok():
                self._stop_requested = True
                self._stop_event.set()
            if not self._stop_requested and failure is None and time.monotonic() >= next_read:
                try:
                    self._read_real_positions(moving=True)
                except Exception as exc:
                    failure = exc
                    # Cancel pending ROS scans before returning control to the
                    # owner loop; an old scan must never overlap a new command.
                    self._stop_requested = True
                    self._stop_event.set()
                next_read = time.monotonic() + 0.1
        worker.join()
        if failure is not None:
            raise failure
        if 'error' in result:
            raise result['error']
        if self._stop_requested:
            return False, "STOP requested."
        return result['value']

    def _confirm_gripper_speed(self, speed):
        """Retry transient speed reads before sending any movement command."""
        if not self.with_gripper:
            raise RuntimeError("Gripper is not installed")
        if self._stop_requested:
            return False
        write_result = self.mc.set_pro_gripper_speed(speed)
        readings = []
        for attempt in range(3):
            if self._stop_requested:
                return False
            try:
                value = self.mc.get_pro_gripper_speed()
            except Exception as exc:
                value = f"{type(exc).__name__}: {exc}"
            readings.append(value)
            if self._stop_requested:
                return False
            valid = (not isinstance(value, bool) and isinstance(value, (int, float))
                     and math.isfinite(value) and 1 <= value <= 100 and value == int(value))
            if valid and value == speed:
                return True
            if attempt < 2:
                self._publish_status(
                    f"Confirming gripper speed {speed}: retry {attempt + 1}/2...")
                # Keep arm feedback current; this uses the cached gripper angle.
                self._read_real_positions(moving=True)
                if self._stop_event.wait(0.1):
                    return False
        raise RuntimeError(
            f"gripper speed could not be confirmed after 3 reads: "
            f"requested={speed}, set_result={write_result!r}, readbacks={readings!r}")

    def _run_real_motion(self, target, speed_scale):
        if self._stop_requested:
            self._publish_status("Cancelled before execution.")
            return
        if "teleop_keyboard_gazebo" in self.get_node_names():
            self._publish_status("Rejected: keyboard controller is running.", level="warning")
            return
        if self.with_gripper:
            self._pose_reader.cached_gripper()
        current = self._read_real_positions(moving=True)
        if not self._gazebo_matches(current):
            if self._stop_requested:
                self._publish_status("Cancelled before execution.")
            else:
                self._publish_status(
                    f"Rejected: Gazebo differs from the real robot by more than {REAL_POSE_TOLERANCE_RAD:.2f} rad.", level="warning"
                )
            return
        while not self._stop_requested:
            valid, reason = self._validate_real_motion(current, target)
            if not valid:
                self._publish_status(f"Rejected: {reason}", level="warning")
                return
            confirmed = self._read_real_positions(moving=True)
            changed = max(abs(a-b) for a,b in zip(confirmed,current)) > 0.01
            current = confirmed
            if not changed:
                break
            self._publish_status("Start pose changed; revalidating before motion...")
        if self._stop_requested:
            self._publish_status("Cancelled before execution.")
            return
        duration = self._trajectory_duration(current, target, speed_scale)
        grip_speed = None
        if self.with_gripper:
            grip_start = int(round(current[6] * 100.0))
            grip_goal = int(round(target[6] * 100.0))
            grip_speed = gripper_speed_for_percentage(grip_start, grip_goal, speed_scale)
        mean_arm_speed = max(abs(b-a) for a,b in zip(current[:6], target[:6])) / duration
        speed = slider_sdk_speed_for_rad(mean_arm_speed)
        if self._stop_requested:
            self._publish_status("Cancelled before execution.")
            return
        self._publish_status(
            f"Executing at {speed_scale * 100:.0f}% speed; "
            f"planned duration {duration:.2f} s."
        )
        if grip_speed is not None:
            if not self._confirm_gripper_speed(grip_speed):
                self._publish_status("Cancelled before execution.")
                return
            self._motion_sent = True
            self._send_gripper(grip_goal)
        if self._stop_requested:
            if self._motion_sent:
                self._stop_real_motion()
            self._publish_status("Cancelled before execution.")
            return
        arm_deg = [math.degrees(value) for value in target[:6]]
        arm_moves = max(abs(a-b) for a,b in zip(current[:6], target[:6])) > WAYPOINT_ARM_TOLERANCE_RAD
        self._motion_sent = True
        result = self.mc.send_angles(arm_deg, speed, _async=True) if arm_moves else 1
        if not sdk_motion_accepted(result):
            self._stop_real_motion()
            self._publish_status(
                f"Real robot command failed: send_angles returned {result!r}", level="error")
            return
        if not self._follow_real_line(
                current, target, duration * 2.0 + REAL_ARRIVAL_MARGIN_S):
            self._stop_real_motion()
            self._publish_status("Execution stopped.")
            return
        self._motion_sent = False
        self._publish_status("Execution complete.")

    def _real_robot_worker(self):
        next_mirror = 0.0
        while ros.ok():
            if self._stop_requested and self._motion_sent:
                try:
                    self._stop_real_motion()
                    self._publish_status("Execution stopped.")
                except Exception as exc:
                    self._publish_status(f"Real robot STOP failed: {exc}", level="error")
                continue
            try:
                target, speed_scale = self.command_queue.get(timeout=0.1)
            except queue.Empty:
                now = time.monotonic()
                if now >= next_mirror:
                    try:
                        self._read_real_positions()
                    except Exception as exc:
                        if now - self._last_read_error >= 2.0:
                            self.get_logger().warning(f"Real feedback unavailable: {exc}")
                            self._last_read_error = now
                    next_mirror = now + 0.25
                continue
            try:
                self._run_real_motion(target, speed_scale)
            except Exception as exc:
                self._publish_status(f"Real robot command failed: {exc}", level="error")
                try:
                    self._stop_real_motion()
                except Exception:
                    self._motion_sent = False
            finally:
                with self._state_lock:
                    self._command_active = False
                self._publish_command_busy()

    def _stop_cb(self, _msg):
        self._stop_requested = True
        self._stop_event.set()
        if self.mode != 2:
            with self._state_lock:
                current = (
                    list(self._current_positions)
                    if self._current_positions is not None
                    else None
                )
                feedback_valid = self._feedback_valid
            limited = self._limited_tracking
            if (
                feedback_valid
                and current is not None
                and all(math.isfinite(value) for value in current)
            ):
                if limited:
                    with self._hold_lock:
                        try:
                            self._set_limited_hold(True)
                        except Exception as exc:
                            self.get_logger().error(f"Could not hold the current pose: {exc}")
                        self._publish_trajectory(current, STOP_HOLD_DURATION, [0.0] * len(current))
                else:
                    self._publish_trajectory(current, MIN_TRAJECTORY_DURATION)
            self._publish_status(
                "STOP applied; controller commanded to hold current position."
            )
            return
        if not self._real_motion_enabled:
            self._publish_status(
                "Simulation STOP applied; real robot remains READ ONLY and received no command."
            )
            return
        self._publish_status(
            "STOP applied; controller commanded to hold current position."
        )


def main(args=None):
    from pro630_ros1 import SignalHandlerOptions
    ros.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    node = None
    try:
        node = SliderControl()
        ros.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Failed to start slider controller: {exc}")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            if node._real_thread is not None:
                node._stop_requested = True
                node._stop_event.set()
                node._real_thread.join(timeout=2.0)
            if node.mc is not None and node._motion_sent:
                try:
                    # Stop motion but keep servo torque enabled; releasing all
                    # servos on process exit could let a loaded arm fall.
                    node.mc.stop()
                except Exception:
                    pass
            if node.mc is not None and hasattr(node.mc, "close"):
                node.mc.close()
            node.destroy_node()
        ros.try_shutdown()


if __name__ == "__main__":
    main()
