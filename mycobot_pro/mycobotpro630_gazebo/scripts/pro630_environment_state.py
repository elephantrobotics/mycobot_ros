#!/usr/bin/env python3
"""Public environment readiness. This process never imports or connects to an SDK."""
import json
import math
import sys
import time

import pro630_ros1 as ros
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from pro630_ros1 import Node
from pro630_ros1 import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Float64MultiArray

from pro630_configuration import configure_node, controllers


class EnvironmentState(Node):
    def __init__(self, announce=False):
        super().__init__('pro630_environment_ready' if announce else 'pro630_environment_state')
        configure_node(self)
        self.declare_parameter('environment', 'simulation')
        self.declare_parameter('session_id', '')
        self.environment = self.get_parameter('environment').value
        if self.allow_collision and self.environment != 'real':
            raise ValueError('allow_collision is supported only by the real environment')
        if self.allow_collision and not announce:
            self.get_logger().warning('allow_collision=true: initial ground collision is permitted; joint motion collision checks remain enabled.')
        self.session = self.get_parameter('session_id').value
        self.announce = announce
        self.verified = False
        self.acknowledged = False
        self.feedback_time = None
        self.collision_ready = False
        self.real_feedback_time = None
        self.real_arm_age = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/pro630/collision_scene_ready',
                                 lambda msg: setattr(self, 'collision_ready', msg.data), qos)
        self.create_subscription(Float64MultiArray, '/pro630/real_feedback_age', self._real_age, 1)
        self.controllers_time = None
        self.controllers_active = False
        self.pending = None
        self.pending_since = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if announce:
            self.pub = self.create_publisher(String, '/pro630/environment_verified', qos)
            self.create_subscription(String, '/pro630/environment_state', self._ack, qos)
        else:
            self.pub = self.create_publisher(String, '/pro630/environment_state', qos)
            self.create_subscription(String, '/pro630/environment_verified', self._verified, qos)
            self.create_subscription(JointState, '/joint_states', self._joints, 1)
            self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
            self.moveit_clients = [self.create_client(kind, name) for kind, name in (
                (GetStateValidity, '/check_state_validity'),
                (GetPlanningScene, '/get_planning_scene'),
                (ApplyPlanningScene, '/apply_planning_scene'))]
        self.create_timer(0.2, self.tick)

    def _verified(self, msg):
        if msg.data == self.session:
            self.verified = True

    def _ack(self, msg):
        try:
            state = json.loads(msg.data)
            self.acknowledged = state['session_id'] == self.session and state['verified']
        except (ValueError, KeyError, TypeError):
            pass

    def _joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions and math.isfinite(positions[name]) for name in self.command_joints):
            self.feedback_time = time.monotonic()

    def _real_age(self, msg):
        if msg.data:
            self.real_feedback_time = time.monotonic()
            self.real_arm_age = msg.data[0]

    def tick(self):
        if self.announce:
            self.pub.publish(String(data=self.session))
            return
        now = time.monotonic()
        if self.pending is not None and self.pending.done():
            try:
                result = self.pending.result()
                active = {c.name for c in result.controller if c.state == 'running'}
                self.controllers_active = set(controllers(self.with_gripper)) <= active
                self.controllers_time = now
            except Exception:
                self.controllers_active = False
            self.pending = None
        if self.pending is not None and now - self.pending_since > 2.0:
            self.pending.cancel()
            self.pending = None
            self.controllers_active = False
        if self.pending is None and self.controllers.service_is_ready():
            self.pending = self.controllers.call_async(ListControllers._request_class())
            self.pending_since = now
        ready = (self.verified and self.collision_ready and self.feedback_time is not None
                 and now - self.feedback_time < 1.0 and self.controllers_active
                 and self.controllers_time is not None and now - self.controllers_time < 2.0
                 and all(c.service_is_ready() for c in self.moveit_clients))
        self.pub.publish(String(data=json.dumps({
            'session_id': self.session, 'environment': self.environment,
            'with_gripper': self.with_gripper, 'allow_collision': self.allow_collision,
            'verified': self.verified, 'ready': bool(ready),
            'tracking_ready': (self.environment == 'simulation' or
                (self.real_feedback_time is not None and now - self.real_feedback_time < 1.
                 and self.real_arm_age is not None and 0 <= self.real_arm_age < .5))})))


def main():
    announce = '--announce-ready' in sys.argv
    ros.init()
    node = EnvironmentState(announce)
    deadline = time.monotonic() + 60.0
    code = 0
    try:
        while ros.ok():
            ros.spin_once(node, timeout_sec=0.2)
            if announce and node.acknowledged:
                break
            if announce and time.monotonic() >= deadline:
                raise RuntimeError('Environment readiness acknowledgement timed out')
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        ros.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
