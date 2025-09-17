#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_gui.py
This module provides a simple Tkinter GUI to control the MyCobot Pro450
robot arm using ROS1 services.

Features:
- Get/set joint angles
- Get/set Cartesian coordinates
- Control gripper open/close
- Set speed

Author: WangWeiJian
Date: 2025-09-08
"""

try:
    import tkinter as tk
    from tkinter import messagebox
except ImportError:
    import Tkinter as tk
    from Tkinter import messagebox
import threading
import rospy
import time
from rospy import ServiceException
from mycobot_pro450_communication.srv import (
    GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus
)


# Joint angle limits
JOINT_LIMITS = [
    (-165, 165),  # joint 1
    (-120, 120),  # joint 2
    (-158, 158),  # joint 3
    (-165, 165),  # joint 4
    (-165, 165),  # joint 5
    (-175, 175),  # joint 6
]

# Coordinate limits
COORD_LIMITS = [
    (-466, 466),   # x
    (-466, 466),   # y
    (-230, 614),   # z
    (-180, 180),   # rx
    (-180, 180),   # ry
    (-180, 180),   # rz
]


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
        self.win.geometry(f"440x440+{x}+{y}")

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

        # Gripper control buttons
        tk.Button(self.frmLB, text="Gripper Open", command=self.gripper_open, width=10).grid(
            row=1, column=0, sticky="w", padx=3, pady=20
        )
        tk.Button(self.frmLB, text="Gripper Close", command=self.gripper_close, width=10).grid(
            row=1, column=1, sticky="w", padx=3, pady=2
        )

        # Uncomment below for vacuum pump control
        # tk.Button(self.frmLB, text="Pump On", command=self.pump_open, width=5).grid(row=2, column=0, sticky="w", padx=3, pady=20)
        # tk.Button(self.frmLB, text="Pump Off", command=self.pump_close, width=5).grid(row=2, column=1, sticky="w", padx=3, pady=2)
        

    def connect_ser(self):
        """Connect to ROS services required for MyCobot control."""
        rospy.init_node("simple_gui", anonymous=True, disable_signals=True)

        rospy.wait_for_service("get_joint_angles")
        rospy.wait_for_service("set_joint_angles")
        rospy.wait_for_service("get_joint_coords")
        rospy.wait_for_service("set_joint_coords")
        rospy.wait_for_service("switch_gripper_status")

        try:
            self.get_coords = rospy.ServiceProxy("get_joint_coords", GetCoords)
            self.set_coords = rospy.ServiceProxy("set_joint_coords", SetCoords)
            self.get_angles = rospy.ServiceProxy("get_joint_angles", GetAngles)
            self.set_angles = rospy.ServiceProxy("set_joint_angles", SetAngles)
            self.switch_gripper = rospy.ServiceProxy("switch_gripper_status", GripperStatus)
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
            tk.Label(self.frmLT, text=f"Joint {i+1}").grid(row=i)

        # Coordinate labels
        coord_names = ["x", "y", "z", "rx", "ry", "rz"]
        for i, name in enumerate(coord_names):
            tk.Label(self.frmRT, text=f" {name} ").grid(row=i)

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
        self.cont_vars = [tk.StringVar(value=f"{val}°") for val in self.res_angles[:6]]
        for i, var in enumerate(self.cont_vars):
            tk.Label(self.frmLC, text=f"Joint {i+1}").grid(row=i)
            tk.Label(self.frmLC, textvariable=var, font=("Arial", 9), width=7, height=1, bg="white").grid(
                row=i, column=1, padx=0, pady=5
            )

        # Coordinate labels and values
        self.coord_vars = [tk.StringVar(value=str(val)) for val in self.record_coords[:6]]
        coord_names = ["x", "y", "z", "rx", "ry", "rz"]
        for i, (name, var) in enumerate(zip(coord_names, self.coord_vars)):
            tk.Label(self.frmLC, text=f"  {name} ").grid(row=i, column=3)
            tk.Label(self.frmLC, textvariable=var, font=("Arial", 9), width=7, height=1, bg="white").grid(
                row=i, column=4, padx=5, pady=5
            )
            tk.Label(self.frmLC, text="mm", font=("Arial", 9)).grid(row=i, column=5)
    
    def gripper_open(self):
        """Open the gripper."""
        try:
            self.switch_gripper(True)
        except ServiceException:
            pass

    def gripper_close(self):
        """Close the gripper."""
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
                    f"{label} {i} out of range!\n"
                    f"Value: {val}, Allowed: {low} ~ {high}"
                )
                return False
        return True

    def validate_speed(self, speed: int) -> bool:
        """Check if speed is within 1–100."""
        if not (1 <= speed <= 100):
            messagebox.showerror(
                "Invalid Speed",
                f"Speed out of range!\n"
                f"Value: {speed}, Allowed: 1 ~ 100"
            )
            return False
        return True

    def get_coord_input(self):
        """Get coordinates from input boxes and send them to the robot."""
        c_value = [float(i.get()) for i in self.c_vars]
        if not self.validate_values(c_value, COORD_LIMITS, "Coordinate"):
            return
        self.speed = int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        if not self.validate_speed(self.speed):
            return
        # c_value.append(self.speed)
        self.async_call(self.set_coords, *c_value, self.speed)
        time.sleep(3)
        self.get_date()
        self.update_display()

    def get_joint_input(self):
        """Get joint angles from input boxes and send them to the robot."""
        j_value = [float(i.get()) for i in self.j_vars]
        if not self.validate_values(j_value, JOINT_LIMITS, "Joint"):
            return
        self.speed = int(float(self.get_speed.get())) if self.get_speed.get() else self.speed
        if not self.validate_speed(self.speed):
            return
        # j_value.append(self.speed)
        # Asynchronous Send
        self.async_call(self.set_angles, *j_value, self.speed)
        time.sleep(3)
        self.get_date()
        self.update_display()

    def async_call(self, func, *args, **kwargs):
        """Run service calls in a separate thread (non-blocking)."""
        threading.Thread(target=func, args=args, kwargs=kwargs, daemon=True).start()
        
    def update_display(self):
        """Update both joint angles and coordinates display."""
        for val, var in zip(self.res_angles, self.cont_vars):
            var.set(f"{val}°")
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
                    round(angles.joint_4, 2), round(angles.joint_5, 2), round(angles.joint_6, 2)
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
            var.set(f"{val}" if way == "coord" else f"{val}°")

    def run(self):
        """Main GUI loop."""
        while True:
            try:
                self.win.update()
                time.sleep(0.001)
            except tk.TclError as e:
                if "application has been destroyed" in str(e):
                    break
                else:
                    raise


def main():
    """Start the MyCobot GUI."""
    window = tk.Tk()
    window.title("MyCobot ROS GUI")
    Window(window).run()


if __name__ == "__main__":
    main()
