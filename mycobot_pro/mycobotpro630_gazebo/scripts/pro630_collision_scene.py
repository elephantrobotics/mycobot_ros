#!/usr/bin/env python3
"""Public owner of the existing single MoveIt floor and collision matrix setup."""
import threading
import pro630_ros1 as ros
from pro630_ros1 import Node
from pro630_ros1 import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from pro630_scene_support import CollisionSceneSupport


class CollisionScene(Node, CollisionSceneSupport):

    def __init__(self):
        super().__init__('pro630_collision_scene')
        self._stop_requested = False
        self.floor_frame, self.floor_size_m = 'world', 20.
        self.floor_thickness_m, self.floor_clearance_m = .10, 0.
        self.get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.pub = self.create_publisher(Bool, '/pro630/collision_scene_ready',
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ready = self.working = False
        self.create_timer(.5, self.tick)

    def tick(self):
        self.pub.publish(Bool(data=self.ready))
        if (self.ready or self.working or not self.get_scene_client.service_is_ready()
                or not self.apply_scene_client.service_is_ready()):
            return
        self.working = True
        threading.Thread(target=self.install_scene, daemon=True).start()

    def install_scene(self):
        try:
            self.ready, reason = self._ensure_floor_collision_scene()
            if not self.ready:
                self.get_logger().warning(reason)
        finally:
            self.working = False


def main():
    ros.init()
    node = CollisionScene()
    try:
        ros.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_requested = True
        node.destroy_node()
        ros.try_shutdown()


if __name__ == '__main__':
    main()
