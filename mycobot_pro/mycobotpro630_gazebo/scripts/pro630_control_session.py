#!/usr/bin/env python3
"""Feature lifecycle only: public environment readiness and owned child processes."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pro630_ros1 as ros
from pro630_ros1 import master_key
from pro630_ros1 import Node
from pro630_ros1 import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from pro630_control_logging import POSITION_LIMIT_SHUTDOWN_S, POSITION_LIMIT_TOPIC
from pro630_sdk_adapter import RobotConnectionLease
from pro630_session_support import EnvironmentLatch, control_commands, stop_owned_children

PACKAGE = 'mycobotpro630_gazebo'


class ControlSession(Node):
    def __init__(self):
        super().__init__('pro630_control_session')
        self.declare_parameter('feature', 'slider')
        self.declare_parameter('environment', 'simulation')
        self.declare_parameter('startup_timeout_sec', 600.0)
        self.declare_parameter('show_force_execute', False)
        self.feature = self.get_parameter('feature').value
        self.environment = self.get_parameter('environment').value
        if self.feature not in ('slider', 'keyboard', 'drag', 'gui') or self.environment not in ('simulation', 'real'):
            raise ValueError('Invalid feature or environment')
        if self.feature in ('drag', 'gui', 'keyboard') and self.environment != 'real':
            raise ValueError(f'{self.feature} requires the real environment')
        self.latch = EnvironmentLatch(self.environment)
        self.failure = None
        self.children = []
        self.lease = None
        self.base_unready_since = None
        self._startup_stale_since = None
        self.position_limit_deadline = None
        self.deadline = time.monotonic() + float(self.get_parameter('startup_timeout_sec').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/pro630/environment_state', self._state, qos)
        if self.environment == 'real':
            self.create_subscription(Bool, POSITION_LIMIT_TOPIC, self._position_limit, qos)
        domain = master_key()
        try:
            self.lease = RobotConnectionLease(f'feature-session-domain-{domain}', 0, timeout=0)
        except Exception as exc:
            raise RuntimeError('Another Pro630 function launch is already running in this ROS master') from exc
        self.get_logger().info(f'{self.feature}: waiting for the public {self.environment} environment.')

    def _state(self, msg):
        try:
            self.latch.observe(json.loads(msg.data))
        except Exception as exc:
            self.failure = str(exc)

    def _position_limit(self, msg):
        if msg.data and self.position_limit_deadline is None:
            self.position_limit_deadline = time.monotonic() + POSITION_LIMIT_SHUTDOWN_S

    def _start_controls(self):
        if {'slider_control_gazebo', 'teleop_keyboard_gazebo', 'pro630_slider_gui', 'pro630_drag_follow', 'pro630_gui_backend', 'pro630_control_gui'} & set(self.get_node_names()):
            raise RuntimeError('A control process is already running; close it before starting this function')
        if self.latch.state.get('allow_collision') is True:
            self.get_logger().warning(f'{self.feature}: allow_collision=true; initial ground collision is permitted; joint motion collision checks remain enabled.')
        directory = Path(__file__).resolve().parent
        for command in control_commands(self.feature, self.environment, directory, sys.executable,
                                        show_force_execute=self.get_parameter('show_force_execute').value,
                                        with_gripper=self.latch.state['with_gripper'],
                                        allow_collision=self.latch.state.get('allow_collision', False)):
            self.children.append(subprocess.Popen(command, start_new_session=True))
        self.get_logger().info(f'{self.feature}: public environment ready; control programs started.')

    def tick(self):
        now = time.monotonic()
        if self.position_limit_deadline is not None:
            return now < self.position_limit_deadline
        if self.failure:
            raise RuntimeError(self.failure)
        if self.children:
            for child in self.children:
                result = child.poll()
                if result is not None:
                    if result != 0:
                        raise RuntimeError(f'Control program exited with code {result}')
                    return False
            if not self.latch.fresh():
                self.get_logger().warning('Environment heartbeat lost; closing this control session.')
                return False
            if not self.latch.ready():
                if self.base_unready_since is None:
                    self.base_unready_since = now
                if now - self.base_unready_since >= 2.0:
                    self.get_logger().warning('Environment unavailable; closing this control session.')
                    return False
            else:
                self.base_unready_since = None
            return True
        if now >= self.deadline:
            raise RuntimeError('Timed out waiting for the public environment')
        if not self.latch.fresh():
            if self.latch.received is not None:
                if self._startup_stale_since is None:
                    self._startup_stale_since = now
                elif now - self._startup_stale_since >= 2.0:
                    raise RuntimeError('Environment heartbeat lost during startup; restart the control launch')
            return True
        self._startup_stale_since = None
        if self.latch.ready() and (self.environment != 'real' or self.latch.state.get('tracking_ready') is True):
            self._start_controls()
        return True

    def close(self):
        try:
            stop_owned_children(self.children)
        finally:
            if self.lease is not None:
                self.lease.close()
                self.lease = None

def interrupted(*_args):
    raise KeyboardInterrupt


def main():
    ros.init()
    node = None
    code = 0
    old_term = signal.signal(signal.SIGTERM, interrupted)
    old_int = signal.signal(signal.SIGINT, interrupted)
    try:
        node = ControlSession()
        while ros.ok():
            ros.spin_once(node, timeout_sec=0.1)
            if not node.tick():
                break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Failed to run Pro630 control session: {exc}', file=sys.stderr)
        code = 1
    finally:
        # launch can send SIGTERM while SIGINT cleanup is still in progress.
        # A second KeyboardInterrupt here would leave detached children alive.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            try:
                node.close()
            except Exception as exc:
                print(f'Pro630 session cleanup failed: {exc}', file=sys.stderr)
                code = 1
            finally:
                node.destroy_node()
        signal.signal(signal.SIGTERM, old_term)
        signal.signal(signal.SIGINT, old_int)
        ros.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
