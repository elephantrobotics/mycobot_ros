"""Shared MoveIt scene installation and service-future helpers."""
import time
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

FLOOR_OBJECT_ID = "pro630_ground_safety"

class CollisionSceneSupport:
    def _wait_for_future(self, future, timeout_sec, operation, *, cancel_on_stop=True):
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            if cancel_on_stop and self._stop_requested:
                return None, "STOP requested."
            time.sleep(0.01)
        if not future.done():
            return None, f"{operation} timed out."
        try:
            response = future.result()
        except Exception as exc:
            return None, f"{operation} failed: {exc}"
        if response is None:
            return None, f"{operation} failed."
        return response, ""


    @staticmethod
    def _ensure_acm_name(acm, name):
        """Add a symmetric row/column to an AllowedCollisionMatrix."""
        if name in acm.entry_names:
            return
        old_size = len(acm.entry_names)
        while len(acm.entry_values) < old_size:
            acm.entry_values.append(AllowedCollisionEntry())
        for row in acm.entry_values[:old_size]:
            if len(row.enabled) < old_size:
                row.enabled.extend([False] * (old_size - len(row.enabled)))
            elif len(row.enabled) > old_size:
                del row.enabled[old_size:]
            row.enabled.append(False)
        new_row = AllowedCollisionEntry()
        new_row.enabled = [False] * (old_size + 1)
        acm.entry_names.append(name)
        acm.entry_values.append(new_row)


    def _ensure_floor_collision_scene(self):
        if not self.get_scene_client.wait_for_service(timeout_sec=3.0):
            return False, "MoveIt /get_planning_scene service is unavailable."
        if not self.apply_scene_client.wait_for_service(timeout_sec=3.0):
            return False, "MoveIt /apply_planning_scene service is unavailable."

        get_request = GetPlanningScene._request_class()
        get_request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        get_future = self.get_scene_client.call_async(get_request)
        get_response, reason = self._wait_for_future(
            get_future, 3.0, "Reading MoveIt allowed-collision matrix"
        )
        if get_response is None:
            return False, reason

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = get_response.scene.allowed_collision_matrix
        acm = scene.allowed_collision_matrix
        # Historical 'Never' exclusions hid gripper-to-wrist contacts. Keep
        # mechanical attachment pairs; enable the movable fingers vs wrist.
        fingers = {f"gripper_{side}{i}" for side in ('left', 'right') for i in (1, 2, 3)}
        for i, left in enumerate(acm.entry_names):
            for j, right in enumerate(acm.entry_names):
                if ((left in fingers and right in ('link5', 'link6')) or
                        (right in fingers and left in ('link5', 'link6'))):
                    acm.entry_values[i].enabled[j] = False
        self._ensure_acm_name(acm, "base")
        self._ensure_acm_name(acm, FLOOR_OBJECT_ID)

        base_index = acm.entry_names.index("base")
        floor_index = acm.entry_names.index(FLOOR_OBJECT_ID)
        acm.entry_values[base_index].enabled[floor_index] = True
        acm.entry_values[floor_index].enabled[base_index] = True
        acm.entry_values[floor_index].enabled[floor_index] = True

        floor = CollisionObject()
        floor.header.frame_id = self.floor_frame
        floor.id = FLOOR_OBJECT_ID
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self.floor_size_m,
            self.floor_size_m,
            self.floor_thickness_m,
        ]
        pose = Pose()
        pose.orientation.w = 1.0
        # The default collision surface matches Gazebo z=0.  A configured
        # positive clearance deliberately turns it into a safety margin.
        pose.position.z = self.floor_clearance_m - self.floor_thickness_m / 2.0
        floor.primitives = [primitive]
        floor.primitive_poses = [pose]
        floor.operation = CollisionObject.ADD
        scene.world.collision_objects = [floor]

        apply_request = ApplyPlanningScene._request_class()
        apply_request.scene = scene
        apply_future = self.apply_scene_client.call_async(apply_request)
        apply_response, reason = self._wait_for_future(
            apply_future, 3.0, "Applying MoveIt ground collision object"
        )
        if apply_response is None:
            return False, reason
        if not apply_response.success:
            return False, "MoveIt rejected the ground collision object."

        self.get_logger().info(
            f"Installed MoveIt ground safety object in '{self.floor_frame}' "
            f"with {self.floor_clearance_m * 1000.0:.0f} mm clearance; "
            "only link 'base' may contact it."
        )
        return True, "ready"

