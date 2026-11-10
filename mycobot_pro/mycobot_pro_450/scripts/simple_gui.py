#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_gui.py
This module provides a simple Tkinter GUI to control the MyCobot Pro450
robot arm using ROS1 services.

F100 gripper Open/Close buttons are shown automatically when
robot_description contains gripper_controller.

Author: WangWeiJian
Date: 2025-09-08
"""

try:
    import tkinter as tk
    from tkinter import messagebox
except ImportError:
    import Tkinter as tk
    from Tkinter import messagebox
import queue
import threading
import rospy
import time
from rospy import ServiceException
from mycobot_pro450_communication.srv import (
    GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus
)
from pymycobot.robot_info import RobotLimit


def detect_f100_gripper():
    """Return True when the loaded URDF exposes gripper_controller."""
    if rospy.has_param("robot_description"):
        return "gripper_controller" in rospy.get_param("robot_description")
    return False


def get_firmware_limits(joint5_offset_deg):
    """Load Pro450 limits from pymycobot's single source of truth."""
    profile = "Pro450ClientNew" if abs(joint5_offset_deg - 90.0) < 0.01 else "Pro450Client"
    try:
        limits = RobotLimit.robot_limit[profile]
        joint_limits = list(zip(limits["angles_min"], limits["angles_max"]))
        coord_limits = list(zip(limits["coords_min"], limits["coords_max"]))
    except (AttributeError, KeyError, TypeError) as exc:
        raise RuntimeError(
            "Unable to load limits for {} from pymycobot: {}"
            .format(profile, exc)
        )
    return joint_limits, coord_limits


class Window:
    """Tkinter GUI window for MyCobot Pro450 ROS1 services control."""

    def __init__(self, handle: tk.Tk):
        """
        Initialize the GUI window, ROS services, and default robot state.

        Args:
            handle: Tkinter root window.
        """
        self.win = handle
        self.win.resizable(0, 0)  # Fixed window size

        # Default speed
        self.speed = rospy.get_param("~speed", 50)
        # GUI fields use firmware joint coordinates; ROS services use URDF
        # logical coordinates, so J5 is translated at this boundary.
        self.joint5_offset_deg = rospy.get_param(
            "/mycobot_pro_450/joint5_offset_deg", 0.0
        )
        self.joint_limits, self.coord_limits = get_firmware_limits(self.joint5_offset_deg)
        self.gripper_enabled = detect_f100_gripper()
        self.speed_d = tk.StringVar()
        self.speed_d.set(str(self.speed))

        # Connect to ROS services
        self.connect_ser()

        # Initialize robot state variables
        self.record_coords = [0, 0, 0, 0, 0, 0, self.speed]
        self.res_angles = [0, 0, 0, 0, 0, 0, self.speed]
        self.get_date()  # Get initial joint angles and coordinates

        # Screen dimensions and window placement
        self.ws = self.win.winfo_screenwidth()
        self.hs = self.win.winfo_screenheight()
        x = int((self.ws / 2) - 190)
        y = int((self.hs / 2) - 250)
        self.win.geometry("600x460+{}+{}".format(x, y))

        # Layout and input/display initialization
        self.set_layout()
        self.need_input()
        self.show_init()

        # Buttons for joint and coordinate settings
        tk.Button(self.frmLT, text="Set Joints", width=10, command=self.get_joint_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )
        tk.Button(self.frmRT, text="Set Coords", width=10, command=self.get_coord_input).grid(
            row=6, column=1, sticky="w", padx=3, pady=2
        )

        if self.gripper_enabled:
            tk.Button(self.frmLB, text="Gripper Open", command=self.gripper_open, width=10).grid(
                row=1, column=0, sticky="w", padx=3, pady=20
            )
            tk.Button(self.frmLB, text="Gripper Close", command=self.gripper_close, width=10).grid(
                row=1, column=1, sticky="w", padx=3, pady=2
            )

        # Queues for thread-safe data transfer
        self.data_queue = queue.Queue()

        # Start a background thread to capture ROS data
        threading.Thread(target=self._refresh_data_thread, daemon=True).start()

        # The main thread refreshes the GUI periodically
        self.win.after(200, self._refresh_gui)

    def _refresh_data_thread(self):
        """Background thread pulls ROS data"""
        while True:
            try:
                self.get_date()  # update self.record_coords and self.res_angles
                # Put it in the queue and pass it to the main thread
                self.data_queue.put((self.record_coords.copy(), self.res_angles.copy()))
            except ServiceException:
                print("ROS service unavailable, skip update.")
            time.sleep(0.1)  # 10Hz

    def _refresh_gui(self):
        """The main thread takes data from the queue and refreshes the interface"""
        try:
            while not self.data_queue.empty():
                coords, angles = self.data_queue.get_nowait()
                self.record_coords = coords
                self.res_angles = angles
        except queue.Empty:
            pass

        # Update display
        self.update_display()

        # Call again after 50ms
        self.win.after(50, self._refresh_gui)

    def connect_ser(self):
        """Connect to ROS services required for MyCobot control."""
        rospy.init_node("simple_gui", anonymous=True, disable_signals=True)

        rospy.wait_for_service("get_joint_angles")
        rospy.wait_for_service("set_joint_angles")
        rospy.wait_for_service("get_joint_coords")
        rospy.wait_for_service("set_joint_coords")
        if self.gripper_enabled:
            rospy.wait_for_service("switch_gripper_status")

        try:
            self.get_coords = rospy.ServiceProxy("get_joint_coords", GetCoords)
            self.set_coords = rospy.ServiceProxy("set_joint_coords", SetCoords)
            self.get_angles = rospy.ServiceProxy("get_joint_angles", GetAngles)
            self.set_angles = rospy.ServiceProxy("set_joint_angles", SetAngles)
            self.switch_gripper = None
            if self.gripper_enabled:
                self.switch_gripper = rospy.ServiceProxy(
                    "switch_gripper_status", GripperStatus
                )
                print("F100 gripper controls enabled.")
            else:
                print("No gripper_controller in robot_description; gripper buttons hidden.")
        except Exception:
            print("Error connecting to services.")
            exit(1)

        print("Connected to ROS services successfully.")

    def set_layout(self):
        """Create frames for GUI layout."""
        self.frmLT = tk.Frame(width=200, height=200)
        self.frmLC = tk.Frame(width=200, height=200)
        self.frmLB = tk.Frame(width=200, height=200)
        self.frmRT = tk.Frame(width=200, height=200)

        self.frmLT.grid(row=0, column=0, padx=1, pady=3)
        self.frmLC.grid(row=1, column=0, padx=1, pady=3)
        self.frmLB.grid(row=1, column=1, padx=2, pady=3)
        self.frmRT.grid(row=0, column=1, padx=2, pady=3)

    def need_input(self):
        """Create input boxes for joints, coordinates, and speed."""
        # Joint labels
        for i in range(6):
            tk.Label(self.frmLT, text="Joint {}".format(i + 1)).grid(row=i)

        # Coordinate labels
        coord_names = ["x", "y", "z", "rx", "ry", "rz"]
        for i, name in enumerate(coord_names):
            tk.Label(self.frmRT, text=" {} ".format(name)).grid(row=i)

        # Initialize StringVars for joint and coordinate inputs
        self.j_vars = [tk.StringVar(value=str(val)) for val in self.res_angles[:6]]
        self.c_vars = [tk.StringVar(value=str(val)) for val in self.record_coords[:6]]

        # Create Entry boxes for joints
        self.j_entries = [tk.Entry(self.frmLT, textvariable=v) for v in self.j_vars]
        for i, entry in enumerate(self.j_entries):
            entry.grid(row=i, column=1, pady=3)

        # Create Entry boxes for coordinates
        self.c_entries = [tk.Entry(self.frmRT, textvariable=v) for v in self.c_vars]
        for i, entry in enumerate(self.c_entries):
            entry.grid(row=i, column=1, pady=3)

        # Speed input
        tk.Label(self.frmLB, text="Speed").grid(row=0, column=0)
        self.get_speed = tk.Entry(self.frmLB, textvariable=self.speed_d, width=10)
        self.get_speed.grid(row=0, column=1)

    def show_init(self):
        """Create display labels for current joint angles and coordinates."""
        # Joint labels and values
        self.cont_vars = [tk.StringVar(value="{}°".format(val)) for val in self.res_angles[:6]]
        for i, var in enumerate(self.cont_vars):
            tk.Label(self.frmLC, text="Joint {}".format(i + 1)).grid(row=i)
            tk.Label(self.frmLC, textvariable=var, font=("Arial", 9), width=7, height=1, bg="white").grid(
                row=i, column=1, padx=0, pady=5
            )

        # Coordinate labels and values
        self.coord_vars = [tk.StringVar(value=str(val)) for val in self.record_coords[:6]]
        coord_names = ["x", "y", "z", "rx", "ry", "rz"]
        for i, (name, var) in enumerate(zip(coord_names, self.coord_vars)):
            tk.Label(self.frmLC, text="  {} ".format(name)).grid(row=i, column=3)
            tk.Label(self.frmLC, textvariable=var, font=("Arial", 9), width=7, height=1, bg="white").grid(
                row=i, column=4, padx=5, pady=5
            )
            tk.Label(self.frmLC, text="mm", font=("Arial", 9)).grid(row=i, column=5)

    def gripper_open(self):
        """Open the gripper."""
        if not self.switch_gripper:
            return
        try:
            self.switch_gripper(True)
        except ServiceException:
            pass

    def gripper_close(self):
        """Close the gripper."""
        if not self.switch_gripper:
            return
        try:
            self.switch_gripper(False)
        except ServiceException:
            pass

    def validate_values(self, values, limits, label):
        """Validate values against limits."""
        for i, (val, (low, high)) in enumerate(zip(values, limits), start=1):
            if not (low <= val <= high):
                messagebox.showerror(
                    "Invalid Input",
                    "{} {} out of range!\nValue: {}, Allowed: {} ~ {}".format(
                        label, i, val, low, high
                    ),
                )
                return False
        return True

    def validate_speed(self, speed: int) -> bool:
        """Check if speed is within 1–100."""
        if not (1 <= speed <= 100):
            messagebox.showerror(
                "Invalid Speed",
                "Speed out of range!\nValue: {}, Allowed: 1 ~ 100".format(speed),
            )
            return False
        return True

    def get_coord_input(self):
        """Get coordinates from input boxes and send them to the robot."""
        c_value = [float(i.get()) for i in self.c_vars]
        if not self.validate_values(c_value, self.coord_limits, "Coordinate"):
            return
        self.speed = int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        if not self.validate_speed(self.speed):
            return
        self.async_call(self.set_coords, *c_value, self.speed)

    def get_joint_input(self):
        """Get joint angles from input boxes and send them to the robot."""
        j_value = [float(i.get()) for i in self.j_vars]
        if not self.validate_values(j_value, self.joint_limits, "Joint"):
            return
        self.speed = int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        if not self.validate_speed(self.speed):
            return
        j_value[4] += self.joint5_offset_deg
        # Asynchronous Send
        self.async_call(self.set_angles, *j_value, self.speed)

    def async_call(self, func, *args, **kwargs):
        """Run service calls in a separate thread (non-blocking)."""
        threading.Thread(target=func, args=args, kwargs=kwargs, daemon=True).start()

    def update_display(self):
        """Update both joint angles and coordinates display."""
        for val, var in zip(self.res_angles, self.cont_vars):
            var.set("{}°".format(val))
        for val, var in zip(self.record_coords[:6], self.coord_vars):
            var.set(str(val))

    def get_date(self):
        """Fetch current robot state (non-blocking)."""
        try:
            # get coordinates
            res = self.get_coords()
            if res:
                self.record_coords = [
                    round(res.x, 2), round(res.y, 2), round(res.z, 2),
                    round(res.rx, 2), round(res.ry, 2), round(res.rz, 2),
                    self.speed
                ]
            else:
                self.record_coords = [-1] * 6
            # get joint angles
            angles = self.get_angles()
            if angles:
                self.res_angles = [
                    round(angles.joint_1, 2), round(angles.joint_2, 2), round(angles.joint_3, 2),
                    round(angles.joint_4, 2), round(angles.joint_5 - self.joint5_offset_deg, 2), round(angles.joint_6, 2)
                ]
            else:
                self.res_angles = [-1] * 6
        except ServiceException:
            print("ROS service unavailable, skip update.")

    def show_j_date(self, data, way: str = ""):
        """Update the display labels with new joint or coordinate data.

        Args:
            data: List of values to display.
            way: "coord" for coordinates, otherwise joint angles.
        """
        target_vars = self.coord_vars if way == "coord" else self.cont_vars
        for val, var in zip(data, target_vars):
            var.set("{}".format(val) if way == "coord" else "{}°".format(val))

    def run(self):
        """Main GUI loop."""
        self.win.mainloop()


def main():
    """Start the MyCobot GUI."""
    window = tk.Tk()
    window.title("MyCobot ROS GUI")
    Window(window).run()


if __name__ == "__main__":
    main()
