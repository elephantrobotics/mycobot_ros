#!/usr/bin/env python3
"""Fail-closed verification of the Gazebo pose seeded from a real snapshot."""

import math
import time

from pro630_configuration import configure_node

import pro630_ros1 as ros
import yaml
from pro630_ros1 import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty




class InitialPoseVerifier(Node):
    def __init__(self):
        super().__init__("pro630_initial_pose_verifier")
        configure_node(self)
        self.declare_parameter("expected_file", "")
        self.declare_parameter("tolerance_rad", 0.01)
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("unpause_before_verify", False)
        self.expected_file = str(self.get_parameter("expected_file").value)
        self.tolerance = max(1.0e-6, float(self.get_parameter("tolerance_rad").value))
        self.deadline = time.monotonic() + max(
            1.0, float(self.get_parameter("timeout_sec").value)
        )
        self.unpause_before_verify = bool(
            self.get_parameter("unpause_before_verify").value
        )
        self.success = False
        self.failure_reason = ""
        self._unpause_requested = False
        self._first_complete_feedback_time = None

        with open(self.expected_file, "r", encoding="utf-8") as handle:
            document = yaml.safe_load(handle)
        initial = document["initial_positions"]
        self.expected = [float(initial[name]) for name in self.command_joints]
        if not all(math.isfinite(value) for value in self.expected):
            raise ValueError("Expected initial pose contains a non-finite value.")

        self.pause_client = self.create_client(Empty, "/gazebo/pause_physics")
        self.unpause_client = self.create_client(Empty, "/gazebo/unpause_physics")
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 20)
        self.create_timer(0.2, self._timer_cb)
        self.get_logger().info(
            f"Verifying Gazebo startup pose against {self.expected_file}."
        )

    def _timer_cb(self):
        if self.success or self.failure_reason:
            return
        if self.unpause_before_verify and not self._unpause_requested:
            if self.unpause_client.wait_for_service(timeout_sec=0.0):
                self.unpause_client.call_async(Empty._request_class())
                self._unpause_requested = True
                self.get_logger().info(
                    "Gazebo controllers are active; unpausing to obtain verified feedback."
                )
        if time.monotonic() >= self.deadline:
            self._fail_and_pause("Timed out waiting for matching Gazebo joint feedback.")

    def _joint_state_cb(self, msg):
        if self.success or self.failure_reason:
            return
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in self.command_joints):
            return
        actual = [float(values[name]) for name in self.command_joints]
        if not all(math.isfinite(value) for value in actual):
            self._fail_and_pause("Gazebo joint feedback contains NaN or infinity.")
            return
        errors = [abs(a - b) for a, b in zip(actual, self.expected)]
        worst_index = max(range(len(errors)), key=errors.__getitem__)
        if errors[worst_index] > self.tolerance:
            now = time.monotonic()
            if self._first_complete_feedback_time is None:
                self._first_complete_feedback_time = now
                return
            # Give gazebo_ros_control one second to publish its seeded state.
            # Do not leave a badly initialized physical model running until the
            # overall timeout expires.
            if now - self._first_complete_feedback_time >= 1.0:
                self._fail_and_pause(
                    f"Gazebo startup pose mismatch: {self.command_joints[worst_index]} error "
                    f"is {errors[worst_index]:.6f} rad (limit {self.tolerance:.6f})."
                )
            return
        self.success = True
        self.get_logger().info(
            "Gazebo startup pose verified against the expected pose; "
            f"maximum error {max(errors):.6f} rad."
        )

    def _fail_and_pause(self, reason):
        self.failure_reason = reason
        self.get_logger().error(reason)

    def pause_physics_and_wait(self):
        if not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Gazebo pause service is unavailable after failure.")
            return
        future = self.pause_client.call_async(Empty._request_class())
        ros.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            self.get_logger().error("Timed out while restoring Gazebo paused state.")


def main(args=None):
    ros.init(args=args)
    node = None
    try:
        node = InitialPoseVerifier()
        while ros.ok() and not node.success and not node.failure_reason:
            ros.spin_once(node, timeout_sec=0.1)
        if node.failure_reason:
            node.pause_physics_and_wait()
        return_code = 0 if node.success else 1
    except (KeyboardInterrupt, OSError, KeyError, TypeError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(f"Initial pose verification failed: {exc}")
        return_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        ros.try_shutdown()
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
