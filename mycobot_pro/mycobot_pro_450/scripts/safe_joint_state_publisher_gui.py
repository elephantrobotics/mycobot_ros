#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Safe JointState slider GUI for Pro450 force-gripper operation.

This intentionally mirrors the useful controls from ROS's
joint_state_publisher_gui (joint sliders, numeric fields, and Center), while
omitting Randomize so a real force gripper cannot be driven to an arbitrary
configuration accidentally.
"""

import math
import sys
import xml.etree.ElementTree as ET

import rospy
from sensor_msgs.msg import JointState

try:
    from python_qt_binding.QtCore import Qt, QTimer
    from python_qt_binding.QtWidgets import (
        QApplication, QAbstractSpinBox, QDoubleSpinBox, QFrame, QGridLayout,
        QLabel, QPushButton, QSlider, QSpinBox, QVBoxLayout, QWidget,
    )
except ImportError:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication, QAbstractSpinBox, QDoubleSpinBox, QFrame, QGridLayout,
        QLabel, QPushButton, QSlider, QSpinBox, QVBoxLayout, QWidget,
    )


SLIDER_STEPS = 10000
PUBLISH_PERIOD_MS = 50
EXPECTED_JOINTS = (
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "gripper_controller",
)


def load_joints():
    """Read controllable joints and limits from robot_description."""
    description = rospy.get_param("robot_description")
    root = ET.fromstring(description)
    discovered = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if not name or joint.get("type") == "fixed" or joint.find("mimic") is not None:
            continue
        limit = joint.find("limit")
        if joint.get("type") == "continuous":
            lower, upper = -math.pi, math.pi
        elif limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        else:
            continue
        discovered[name] = (lower, upper)

    joints = [(name, *discovered[name]) for name in EXPECTED_JOINTS if name in discovered]
    if len(joints) != len(EXPECTED_JOINTS):
        rospy.logwarn(
            "安全滑块未找到预期的全部 Pro450 关节，使用 URDF 中的可控关节 / "
            "Safe slider is using all controllable URDF joints"
        )
        joints = [(name, *limits) for name, limits in discovered.items()]
    if not joints:
        raise RuntimeError("robot_description 中没有可控关节 / No controllable joints in robot_description")
    return joints


class SafeJointStatePublisher(QWidget):
    """A no-randomize JointState GUI."""

    def __init__(self, joints):
        super().__init__()
        self.joints = joints
        self.values = [min(max(0.0, lower), upper) for _, lower, upper in joints]
        self.sliders = []
        self.spins = []
        self.publisher = rospy.Publisher("joint_states", JointState, queue_size=10)

        self.setWindowTitle("Node: /joint_state_publisher_gui (safe)")
        self.setFixedWidth(360)
        self._build_layout()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish)
        self.timer.start(PUBLISH_PERIOD_MS)

    def _build_layout(self):
        outer = QVBoxLayout(self)
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel_layout = QVBoxLayout(panel)
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(2)
        grid.setColumnStretch(1, 1)
        panel_layout.addLayout(grid)

        for row, (name, lower, upper) in enumerate(self.joints):
            grid_row = row * 2
            label = QLabel(name)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, SLIDER_STEPS)
            slider.setValue(self._slider_value(row, self.values[row]))

            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setRange(lower, upper)
            spin.setSingleStep((upper - lower) / 100.0)
            spin.setValue(self.values[row])
            # Match ROS's broad value-entry field rather than the compact
            # spin box styling used by the first safe-GUI version.
            spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
            # Let Qt allocate the field width according to the label.  The
            # gripper_controller label is longer than joint1..joint6.
            spin.setMinimumWidth(0)

            slider.valueChanged.connect(lambda value, index=row: self._slider_changed(index, value))
            spin.valueChanged.connect(lambda value, index=row: self._spin_changed(index, value))

            # The stock GUI puts the name/value above a full-width slider.
            grid.addWidget(label, grid_row, 0)
            grid.addWidget(spin, grid_row, 1, 1, 2)
            grid.addWidget(slider, grid_row + 1, 0, 1, 3)
            self.sliders.append(slider)
            self.spins.append(spin)

        panel_layout.addStretch(1)
        center = QPushButton("Center")
        center.clicked.connect(self.center)
        panel_layout.addWidget(center)

        # The stock GUI displays the number of active slider controls below
        # Center.  It is informational here: the URDF determines the count.
        joint_count = QSpinBox()
        joint_count.setRange(len(self.joints), len(self.joints))
        joint_count.setValue(len(self.joints))
        joint_count.setReadOnly(True)
        panel_layout.addWidget(joint_count)
        outer.addWidget(panel)

    def _slider_value(self, index, value):
        _, lower, upper = self.joints[index]
        return int(round((value - lower) * SLIDER_STEPS / (upper - lower)))

    def _joint_value(self, index, slider_value):
        _, lower, upper = self.joints[index]
        return lower + (upper - lower) * slider_value / float(SLIDER_STEPS)

    def _slider_changed(self, index, slider_value):
        value = self._joint_value(index, slider_value)
        self.values[index] = value
        self.spins[index].blockSignals(True)
        self.spins[index].setValue(value)
        self.spins[index].blockSignals(False)

    def _spin_changed(self, index, value):
        self.values[index] = value
        self.sliders[index].blockSignals(True)
        self.sliders[index].setValue(self._slider_value(index, value))
        self.sliders[index].blockSignals(False)

    def center(self):
        for index, (_, lower, upper) in enumerate(self.joints):
            self.spins[index].setValue(min(max(0.0, lower), upper))

    def publish(self):
        message = JointState()
        message.header.stamp = rospy.Time.now()
        message.name = [name for name, _, _ in self.joints]
        message.position = list(self.values)
        self.publisher.publish(message)


def main():
    rospy.init_node("safe_joint_state_publisher_gui")
    joints = load_joints()
    use_gui = rospy.get_param("~use_gui", True)
    if not use_gui:
        publisher = rospy.Publisher("joint_states", JointState, queue_size=10)
        values = [min(max(0.0, lower), upper) for _, lower, upper in joints]
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            message = JointState()
            message.header.stamp = rospy.Time.now()
            message.name = [name for name, _, _ in joints]
            message.position = values
            publisher.publish(message)
            rate.sleep()
        return

    app = QApplication(sys.argv)
    window = SafeJointStatePublisher(joints)
    rospy.on_shutdown(app.quit)
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
