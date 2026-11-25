"""Shared read-only MoveIt collision checks; no scene installation or SDK access."""
import math
import threading
import time

from moveit_msgs.srv import GetStateValidity

COLLISION_SAMPLE_STEP = math.radians(1.0)
MAX_COLLISION_SAMPLES = 400
DEFAULT_COLLISION_DEPTH_TOLERANCE_M = 0.0001
SHOW_COLLISION_PATH_PERCENT = False
SHOW_COLLISION_DEPTH = False
SHOW_COLLISION_BODIES = False


class CollisionCheck:
    """Requires a validity client, active joints, cancellation flag and depth trend tolerance."""

    def _path_is_valid(self, current, target):
        if not self.validity_client.wait_for_service(timeout_sec=2.0):
            return False, "MoveIt /check_state_validity service is unavailable."

        max_delta = max(abs(b - a) for a, b in zip(current, target))
        sample_count = max(
            2, min(MAX_COLLISION_SAMPLES, math.ceil(max_delta / COLLISION_SAMPLE_STEP))
        )

        current_valid, previous_contacts, reason = self._state_is_valid(current)
        if reason:
            return False, reason
        escaping_collision = not current_valid
        initial_contacts = dict(previous_contacts)

        for index in range(1, sample_count + 1):
            if self._stop_requested:
                return False, "STOP requested."
            ratio = index / sample_count
            sample = [a + (b - a) * ratio for a, b in zip(current, target)]

            valid, contacts, reason = self._state_is_valid(sample)
            if reason:
                return False, reason
            if valid:
                escaping_collision = False
                previous_contacts = {}
                continue

            if escaping_collision and self._collision_is_decreasing(
                initial_contacts,
                previous_contacts,
                contacts,
            ):
                previous_contacts = contacts
                continue

            return False, self._format_collision_reason(ratio, contacts)

        if escaping_collision:
            return False, (
                "target remains in collision; move farther along a path that "
                "fully exits the existing contact."
            )

        return True, "valid"


    def _state_is_valid(self, positions):
        """Return validity and maximum penetration depth for every body pair."""

        request = GetStateValidity._request_class()
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(self.command_joints)
        request.robot_state.joint_state.position = positions
        # Empty group checks the entire robot, including the separate gripper.
        request.group_name = ""

        future = self.validity_client.call_async(request)
        reply_ready = threading.Event()
        future.add_done_callback(lambda _future: reply_ready.set())
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            if self._stop_requested:
                future.cancel()
                return False, {}, "STOP requested."
            # Wake as soon as ROS supplies the result. Sleeping per sample adds
            # seconds to a dense keyboard scan despite sub-millisecond replies.
            reply_ready.wait(min(0.01, max(0., deadline - time.monotonic())))
        if not future.done():
            future.cancel()
            return False, {}, "MoveIt state-validity check timed out."
        response = future.result()
        if response is None:
            return False, {}, "MoveIt state-validity check failed."
        if response.valid:
            return True, {}, ""

        contacts = {}
        for contact in response.contacts:
            # MoveIt already decided this state is invalid. Contact depth,
            # including zero, must not turn that result back into a pass.
            # Mounting contacts stay in the allowed-collision matrix.
            pair = tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
            contacts[pair] = max(contacts.get(pair, 0.0), contact.depth)

        if not contacts:
            return False, {}, "MoveIt reported an invalid state without contacts."
        return False, contacts, ""


    def _collision_is_decreasing(self, initial, previous, current):
        if not set(current).issubset(initial):
            return False
        tolerance = self.collision_depth_tolerance_m
        for pair, depth in current.items():
            if depth > initial[pair] + tolerance:
                return False
            if depth > previous.get(pair, 0.0) + tolerance:
                return False
        return True


    @staticmethod
    def _format_collision_reason(ratio, contacts):
        if not contacts:
            if SHOW_COLLISION_PATH_PERCENT:
                return f"invalid state at {ratio * 100:.0f}% of path."
            return "invalid state."

        pair, depth = max(contacts.items(), key=lambda item: item[1])
        summary = "collision detected"
        if SHOW_COLLISION_PATH_PERCENT:
            summary += f" at {ratio * 100:.0f}% of path"

        details = []
        if SHOW_COLLISION_BODIES:
            details.append(f"{pair[0]} vs {pair[1]}")
        if SHOW_COLLISION_DEPTH:
            details.append(f"depth {depth * 1000.0:.3f} mm")
        return f"{summary} ({', '.join(details)})." if details else f"{summary}."

