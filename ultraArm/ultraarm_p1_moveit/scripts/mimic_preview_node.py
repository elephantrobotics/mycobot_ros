#!/usr/bin/env python3
"""Publish mimic-expanded joint states for MoveIt Goal / Planned-Path preview.

MoveIt's MotionPlanning display often fails to sync parallel-link mimic joints.
This node:
  - listens to Interactive Marker feedback/update (Marker drag + Joints tab)
  - solves IK for Goal preview (seeded from last goal to track Joints sliders)
  - listens to DisplayTrajectory and plays back waypoints for path preview
  - publishes joints on /mimic_preview/joint_states for a prefixed RSP
"""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

ACTIVE_JOINTS = ("J1", "J2", "J3", "J4")
# URDF mimic: child = multiplier * parent + offset (radians)
MIMIC_JOINTS = {
    "center": ("J2", -1.0, 0.0),
    "CENTER_V_2": ("J3", 1.0, 0.0),
    "J3_4": ("J3", -1.0, 0.0),
    "J3_5": ("J3", -1.0, 0.0),
    "J2_2": ("J2", 1.0, 0.0),
    "center_v_1": ("J2", -1.0, 0.0),
}
J2_RANGE = (-18.0, 85.0)
J3_RANGE = (-1.0, 110.0)
ZERO_EPS_DEG = 0.1


def snap_zero(angle_deg):
    return 0.0 if abs(angle_deg) < ZERO_EPS_DEG else angle_deg


def valid_region(j2_deg, j3_deg):
    """Same J2/J3 coupling check as joint_coupling_node (incl. 42° hard cut)."""
    a = snap_zero(j2_deg)
    b = snap_zero(j3_deg)
    if not (J2_RANGE[0] <= a <= J2_RANGE[1] and J3_RANGE[0] <= b <= J3_RANGE[1]):
        return False
    if -18 <= a < 0:
        if b >= 42.0:
            return False
        cond1 = math.cos(math.radians(-a + b)) - math.sin(math.radians(45 + a)) <= 7 / 30
        cond2 = abs(math.cos(math.radians(-a + b))) >= 15.4 / 30
        return cond1 and cond2
    if 0 <= a <= 50.87:
        return math.cos(math.radians(a - b)) >= 15.4 / 30
    if 50.87 < a < 76.72:
        return True
    if 76.72 <= a <= 85:
        return abs(math.cos(math.radians(a - b))) >= 6.89 / 30
    return False


def active_j2_j3_deg(active):
    return (
        math.degrees(float(active["J2"])),
        math.degrees(float(active["J3"])),
    )


def _pose_changed(a, b, lin_eps=1e-4, ang_eps=1e-3):
    if a is None or b is None:
        return True
    dp = a.position
    dq = b.position
    if abs(dp.x - dq.x) > lin_eps or abs(dp.y - dq.y) > lin_eps or abs(dp.z - dq.z) > lin_eps:
        return True
    # Quaternion absolute dot (q and -q are same rotation)
    oa, ob = a.orientation, b.orientation
    dot = abs(oa.x * ob.x + oa.y * ob.y + oa.z * ob.z + oa.w * ob.w)
    return dot < math.cos(ang_eps / 2.0)


class MimicPreviewNode(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._current = {name: 0.0 for name in ACTIVE_JOINTS}
        self._goal_active = {name: 0.0 for name in ACTIVE_JOINTS}
        self._mode = "goal"  # goal | trajectory
        self._user_preview = False
        self._traj_points = []
        self._traj_index = 0
        self._last_ee_pose = None
        self._last_ik_time = rospy.Time(0)
        self._last_invalid_pair = None
        self._was_invalid = False
        self._group = rospy.get_param("~planning_group", "arm_group")
        self._ee_link = rospy.get_param("~ee_link", "J4_Link")
        self._ik_timeout = float(rospy.get_param("~ik_timeout", 0.05))
        self._state_dt = float(rospy.get_param("~state_display_time", 0.05))
        self._ik_min_period = float(rospy.get_param("~ik_min_period", 0.05))
        self._marker_base = rospy.get_param(
            "~marker_topic_base",
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic",
        )

        self._pub = rospy.Publisher(
            "/mimic_preview/joint_states", JointState, queue_size=10
        )
        rospy.Subscriber("/joint_states", JointState, self._on_joint_states, queue_size=1)
        rospy.Subscriber(
            "move_group/display_planned_path",
            DisplayTrajectory,
            self._on_display_trajectory,
            queue_size=1,
        )
        rospy.Subscriber(
            self._marker_base + "/feedback",
            InteractiveMarkerFeedback,
            self._on_marker_feedback,
            queue_size=10,
        )
        # Joints tab updates EE marker via server setPose -> /update (no POSE_UPDATE feedback)
        rospy.Subscriber(
            self._marker_base + "/update",
            InteractiveMarkerUpdate,
            self._on_marker_update,
            queue_size=10,
        )

        rospy.wait_for_service("/compute_ik", timeout=30.0)
        self._ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        self._timer = rospy.Timer(rospy.Duration(self._state_dt), self._on_timer)
        rospy.loginfo(
            "mimic_preview_node ready (group=%s, ee=%s, marker=%s)",
            self._group,
            self._ee_link,
            self._marker_base,
        )

    def _on_joint_states(self, msg):
        with self._lock:
            for name in ACTIVE_JOINTS:
                if name in msg.name:
                    self._current[name] = msg.position[msg.name.index(name)]
            if not self._user_preview and self._mode == "goal":
                self._goal_active = dict(self._current)

    def _expand(self, active):
        names = list(ACTIVE_JOINTS)
        positions = [float(active[name]) for name in ACTIVE_JOINTS]
        for child, (parent, mult, offset) in MIMIC_JOINTS.items():
            names.append(child)
            positions.append(mult * float(active[parent]) + offset)
        return names, positions

    def _publish_active(self, active):
        names, positions = self._expand(active)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = positions
        self._pub.publish(msg)

    def _active_from_trajectory_point(self, point, joint_names):
        active = {name: 0.0 for name in ACTIVE_JOINTS}
        for name in ACTIVE_JOINTS:
            if name in joint_names:
                active[name] = point.positions[joint_names.index(name)]
        return active

    def _on_display_trajectory(self, msg):
        if not msg.trajectory:
            return
        robot_traj = msg.trajectory[0]
        joint_names = list(robot_traj.joint_trajectory.joint_names)
        points = list(robot_traj.joint_trajectory.points)
        if not points or not joint_names:
            return
        with self._lock:
            traj = []
            for p in points:
                active = self._active_from_trajectory_point(p, joint_names)
                j2, j3 = active_j2_j3_deg(active)
                if not valid_region(j2, j3):
                    rospy.logwarn(
                        "mimic preview: planned path leaves coupled region at "
                        "J2=%.2f J3=%.2f deg; truncating playback",
                        j2,
                        j3,
                    )
                    break
                traj.append(active)
            if not traj:
                rospy.logwarn("mimic preview: no valid waypoints in planned path")
                return
            self._traj_points = traj
            self._traj_index = 0
            self._mode = "trajectory"
            self._user_preview = True
        rospy.loginfo(
            "mimic preview: playing planned path (%d/%d waypoints)",
            len(traj),
            len(points),
        )

    def _ik_seed(self):
        """Prefer last goal so Joints-tab tracking stays continuous."""
        with self._lock:
            if self._user_preview:
                seed_joints = dict(self._goal_active)
            else:
                seed_joints = dict(self._current)
        state = RobotState()
        state.joint_state.name = list(ACTIVE_JOINTS)
        state.joint_state.position = [seed_joints[name] for name in ACTIVE_JOINTS]
        state.is_diff = True
        return state, seed_joints

    def _is_ee_marker(self, name):
        lower = name.lower()
        return (
            "ee" in lower
            or "goal" in lower
            or self._ee_link.lower() in lower
            or "arm_group" in lower
        )

    def _apply_ee_pose(self, header, pose, force=False):
        # Always de-dupe: /update streams the same illegal goal repeatedly while
        # the marker sits outside the coupled region.
        if not _pose_changed(self._last_ee_pose, pose):
            return
        now = rospy.Time.now()
        if (now - self._last_ik_time).to_sec() < self._ik_min_period:
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = header
        if not pose_stamped.header.frame_id:
            pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = pose

        seed, _seed_joints = self._ik_seed()
        req = GetPositionIKRequest()
        req.ik_request.group_name = self._group
        req.ik_request.robot_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(self._ik_timeout)
        req.ik_request.ik_link_name = self._ee_link

        try:
            res = self._ik(req)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "compute_ik failed: %s", exc)
            return

        if res.error_code.val != res.error_code.SUCCESS:
            return

        active = dict(_seed_joints)
        js = res.solution.joint_state
        for name in ACTIVE_JOINTS:
            if name in js.name:
                active[name] = js.position[js.name.index(name)]

        j2, j3 = active_j2_j3_deg(active)
        self._last_ee_pose = pose
        self._last_ik_time = now
        if not valid_region(j2, j3):
            # Keep last valid preview; warn only on enter / significant change.
            should_warn = not self._was_invalid
            if self._last_invalid_pair is not None:
                should_warn = should_warn or abs(j2 - self._last_invalid_pair[0]) >= 0.5
                should_warn = should_warn or abs(j3 - self._last_invalid_pair[1]) >= 0.5
            if should_warn:
                rospy.logwarn(
                    "mimic preview: reject uncoupled J2/J3 (%.2f, %.2f deg)",
                    j2,
                    j3,
                )
                self._last_invalid_pair = (j2, j3)
            self._was_invalid = True
            return

        self._was_invalid = False
        self._last_invalid_pair = None
        with self._lock:
            self._mode = "goal"
            self._user_preview = True
            self._traj_points = []
            self._goal_active = active
        self._publish_active(active)

    def _on_marker_feedback(self, msg):
        if msg.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
            InteractiveMarkerFeedback.KEEP_ALIVE,
        ):
            return
        if msg.marker_name and not self._is_ee_marker(msg.marker_name):
            return
        force = msg.event_type != InteractiveMarkerFeedback.KEEP_ALIVE
        self._apply_ee_pose(msg.header, msg.pose, force=force)

    def _on_marker_update(self, msg):
        # Joints tab / named goal: server publishes pose updates here
        entries = [(p.name, p.header, p.pose) for p in msg.poses]
        entries.extend((m.name, m.header, m.pose) for m in msg.markers)
        matched = [
            (name, header, pose)
            for name, header, pose in entries
            if not name or self._is_ee_marker(name)
        ]
        if not matched and entries:
            matched = entries[:1]
        for _name, header, pose in matched:
            self._apply_ee_pose(header, pose, force=True)

    def _on_timer(self, _event):
        with self._lock:
            mode = self._mode
            if mode == "trajectory" and self._traj_points:
                active = self._traj_points[self._traj_index]
                self._traj_index += 1
                if self._traj_index >= len(self._traj_points):
                    self._mode = "goal"
                    self._goal_active = dict(active)
                    self._traj_points = []
            else:
                active = dict(self._goal_active)
        self._publish_active(active)


def main():
    rospy.init_node("mimic_preview_node")
    MimicPreviewNode()
    rospy.spin()


if __name__ == "__main__":
    main()
