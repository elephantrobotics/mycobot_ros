#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""Coupling-aware joint slider GUI for ultraArm P1 (coupling_slider.launch).

Publishes /joint_states_raw like joint_state_publisher_gui, but:
- Randomize only samples valid J2-J3 pairs
- Dragging into the forbidden region snaps back to the last valid pose

UI mimics joint_state_publisher_gui (blue filled track + full-width buttons).
"""

from __future__ import print_function

import math
import random
import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String

try:
    import tkinter as tk
    from tkinter import messagebox
except ImportError:
    import Tkinter as tk
    import tkMessageBox as messagebox

# Model-angle limits (deg), aligned with URDF / joint_coupling_node.
JOINT_LIMITS_DEG = {
    "J1": (-164.4, 164.4),
    "J2": (-18.0, 85.0),
    "J3": (-1.0, 110.0),
    "J4": (-178.8, 178.8),
}
JOINT_NAMES = ["J1", "J2", "J3", "J4"]
J2_RANGE = JOINT_LIMITS_DEG["J2"]
J3_RANGE = JOINT_LIMITS_DEG["J3"]
ZERO_EPS_DEG = 0.1
RANDOM_MAX_TRIES = 5000

# Visual style close to joint_state_publisher_gui (Qt slider look).
TRACK_BG = "#dcdcdc"
TRACK_FILL = "#3d7ea6"
THUMB_EDGE = "#666666"
PANEL_BG = "#f0f0f0"


def snap_zero(angle_deg):
    return 0.0 if abs(angle_deg) < ZERO_EPS_DEG else angle_deg


def valid_region(a, b):
    """Same J2-J3 coupling check as joint_coupling_node (model deg)."""
    a = snap_zero(a)
    b = snap_zero(b)
    if not (J2_RANGE[0] <= a <= J2_RANGE[1] and J3_RANGE[0] <= b <= J3_RANGE[1]):
        return False
    if -18 <= a < 0:
        if b >= 42.0:
            return False
        cond1 = math.cos(math.radians(-a + b)) - math.sin(math.radians(45 + a)) <= 7.0 / 30.0
        cond2 = abs(math.cos(math.radians(-a + b))) >= 15.4 / 30.0
        return cond1 and cond2
    if 0 <= a <= 50.87:
        return math.cos(math.radians(a - b)) >= 15.4 / 30.0
    if 50.87 < a < 76.72:
        return True
    if 76.72 <= a <= 85:
        return abs(math.cos(math.radians(a - b))) >= 6.89 / 30.0
    return False


def sample_valid_j2_j3():
    lo2, hi2 = J2_RANGE
    lo3, hi3 = J3_RANGE
    for _ in range(RANDOM_MAX_TRIES):
        j2 = random.uniform(lo2, hi2)
        j3 = random.uniform(lo3, hi3)
        if valid_region(j2, j3):
            return round(j2, 2), round(j3, 2)
    return None


class FilledSlider(tk.Frame):
    """Horizontal slider with blue fill, similar to joint_state_publisher_gui."""

    def __init__(self, master, from_, to, variable, command=None, length=320, height=18, **kwargs):
        tk.Frame.__init__(self, master, bg=PANEL_BG, **kwargs)
        self.from_ = float(from_)
        self.to = float(to)
        self.variable = variable
        self.command = command
        self.length = int(length)
        self.height = int(height)
        self._drag = False
        self.canvas = tk.Canvas(
            self,
            width=self.length,
            height=self.height,
            bg=PANEL_BG,
            highlightthickness=0,
            bd=0,
        )
        self.canvas.pack(fill="x", expand=True)
        self.canvas.bind("<Configure>", self._on_configure)
        self.canvas.bind("<Button-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        try:
            self.variable.trace_add("write", lambda *_a: self._redraw())
        except AttributeError:
            self.variable.trace("w", lambda *_a: self._redraw())
        self.after_idle(self._redraw)

    def _span(self):
        return self.to - self.from_ if self.to != self.from_ else 1.0

    def _value_to_x(self, value):
        w = max(int(self.canvas.winfo_width()), self.length)
        ratio = (float(value) - self.from_) / self._span()
        ratio = 0.0 if ratio < 0.0 else 1.0 if ratio > 1.0 else ratio
        return ratio * (w - 1)

    def _x_to_value(self, x):
        w = max(int(self.canvas.winfo_width()), self.length)
        ratio = float(x) / float(max(w - 1, 1))
        ratio = 0.0 if ratio < 0.0 else 1.0 if ratio > 1.0 else ratio
        return self.from_ + ratio * self._span()

    def _on_configure(self, _event):
        self._redraw()

    def _on_press(self, event):
        self._drag = True
        self._set_from_x(event.x)

    def _on_drag(self, event):
        if self._drag:
            self._set_from_x(event.x)

    def _on_release(self, _event):
        self._drag = False

    def _set_from_x(self, x):
        value = round(self._x_to_value(x), 2)
        self.variable.set(value)
        self._redraw()
        if self.command:
            self.command(str(value))

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        w = max(int(c.winfo_width()), self.length)
        h = self.height
        pad = 2
        c.create_rectangle(0, pad, w, h - pad, fill=TRACK_BG, outline="#b0b0b0")
        x = self._value_to_x(self.variable.get())
        if x > 0:
            c.create_rectangle(0, pad, x, h - pad, fill=TRACK_FILL, outline=TRACK_FILL)
        c.create_line(x, pad - 1, x, h - pad + 1, fill=THUMB_EDGE, width=2)


class CouplingSliderGui(object):
    def __init__(self):
        self.pub = rospy.Publisher("/joint_states_raw", JointState, queue_size=10)
        # Same topic as joint_coupling_node -> coupling_warn_gui (Option 1 popup).
        self.warn_pub = rospy.Publisher(
            "/ultraarm_p1/j2_j3_coupling_warning", String, queue_size=1
        )
        self._applying = False
        self._snap_warned = False
        self.angles_deg = {name: 0.0 for name in JOINT_NAMES}
        self.last_valid_deg = dict(self.angles_deg)

        self.root = tk.Tk()
        self.root.withdraw()
        self.root.title("Joint State Publisher")
        self.root.configure(bg=PANEL_BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        frm = tk.Frame(self.root, bg=PANEL_BG, padx=8, pady=8)
        frm.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        self.vars = {}
        self.scales = {}
        label_font = ("Sans", 10)
        value_font = ("Sans", 10)

        for i, name in enumerate(JOINT_NAMES):
            lo, hi = JOINT_LIMITS_DEG[name]
            tk.Label(
                frm, text=name, bg=PANEL_BG, font=label_font, width=4, anchor="w"
            ).grid(row=i, column=0, sticky="w", pady=3)
            var = tk.DoubleVar(value=0.0)
            self.vars[name] = var
            scale = FilledSlider(
                frm,
                from_=lo,
                to=hi,
                variable=var,
                length=340,
                height=18,
                command=lambda _v, n=name: self._on_slider(n),
            )
            scale.grid(row=i, column=1, sticky="ew", padx=6, pady=3)
            val_lbl = tk.Label(
                frm, text="0.00", bg=PANEL_BG, font=value_font, width=7, anchor="e"
            )
            val_lbl.grid(row=i, column=2, sticky="e", pady=3)
            self.scales[name] = (scale, val_lbl)

        btn_font = ("Sans", 10)
        btn_row = len(JOINT_NAMES)
        tk.Button(
            frm, text="Randomize", font=btn_font, command=self._randomize, pady=4
        ).grid(row=btn_row, column=0, columnspan=3, sticky="ew", pady=(10, 4))
        tk.Button(
            frm, text="Center", font=btn_font, command=self._center, pady=4
        ).grid(row=btn_row + 1, column=0, columnspan=3, sticky="ew", pady=(0, 4))

        # Match JSP footer style (joint count).
        tk.Label(
            frm, text="4", bg=PANEL_BG, fg="#444444", font=("Sans", 9), anchor="w"
        ).grid(row=btn_row + 2, column=0, columnspan=3, sticky="w", pady=(2, 0))

        self._publish()
        self.root.update_idletasks()
        self._center_window()
        self.root.deiconify()
        self.root.after(33, self._tick)

    def _center_window(self):
        self.root.update_idletasks()
        w = self.root.winfo_reqwidth()
        h = self.root.winfo_reqheight()
        x = max((self.root.winfo_screenwidth() - w) // 2, 0)
        y = max((self.root.winfo_screenheight() - h) // 2, 0)
        self.root.geometry("%dx%d+%d+%d" % (w, h, x, y))

    def _on_close(self):
        rospy.signal_shutdown("coupling_slider_gui closed")
        self.root.quit()

    def _publish_coupling_warn(self, j2, j3):
        """Drive Option-1 coupling_warn_gui with the same message format."""
        warn_text = (
            "Invalid J2-J3 combination: "
            "J2=%.2f deg (%.3f rad), J3=%.2f deg (%.3f rad)"
            % (j2, math.radians(j2), j3, math.radians(j3))
        )
        self.warn_pub.publish(String(data=warn_text))
        self._snap_warned = True

    def _clear_coupling_warn(self):
        if self._snap_warned:
            self.warn_pub.publish(String(data="ok"))
            self._snap_warned = False

    def _read_scales(self):
        return {name: float(self.vars[name].get()) for name in JOINT_NAMES}

    def _apply_angles(self, angles_deg, update_last_valid=True):
        self._applying = True
        try:
            for name in JOINT_NAMES:
                val = float(angles_deg[name])
                self.vars[name].set(val)
                _scale, lbl = self.scales[name]
                lbl.configure(text="%.2f" % val)
                self.angles_deg[name] = val
            if update_last_valid:
                self.last_valid_deg = dict(self.angles_deg)
        finally:
            self._applying = False
        self._publish()

    def _on_slider(self, changed_name):
        if self._applying or rospy.is_shutdown():
            return
        angles = self._read_scales()
        for name in JOINT_NAMES:
            _scale, lbl = self.scales[name]
            lbl.configure(text="%.2f" % angles[name])

        j2, j3 = angles["J2"], angles["J3"]
        if not valid_region(j2, j3):
            # Snap back; notify via coupling_warn_gui (same as Option 1).
            self._apply_angles(self.last_valid_deg, update_last_valid=False)
            self._publish_coupling_warn(j2, j3)
            return

        self._clear_coupling_warn()
        self.angles_deg = angles
        self.last_valid_deg = dict(angles)
        self._publish()

    def _randomize(self):
        if self._applying:
            return
        pair = sample_valid_j2_j3()
        if pair is None:
            messagebox.showerror(
                "Randomize failed",
                "Could not sample a valid J2-J3 pair. Keeping current pose.",
                parent=self.root,
            )
            return
        j2, j3 = pair
        lo1, hi1 = JOINT_LIMITS_DEG["J1"]
        lo4, hi4 = JOINT_LIMITS_DEG["J4"]
        angles = {
            "J1": round(random.uniform(lo1, hi1), 2),
            "J2": j2,
            "J3": j3,
            "J4": round(random.uniform(lo4, hi4), 2),
        }
        self._clear_coupling_warn()
        self._apply_angles(angles, update_last_valid=True)

    def _center(self):
        zeros = {name: 0.0 for name in JOINT_NAMES}
        if not valid_region(zeros["J2"], zeros["J3"]):
            return
        self._clear_coupling_warn()
        self._apply_angles(zeros, update_last_valid=True)
    def _publish(self):
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = list(JOINT_NAMES)
        js.position = [math.radians(self.angles_deg[n]) for n in JOINT_NAMES]
        self.pub.publish(js)

    def _tick(self):
        if rospy.is_shutdown():
            try:
                self.root.quit()
            except Exception:
                pass
            return
        self._publish()
        self.root.after(33, self._tick)

    def spin(self):
        self.root.mainloop()


def main():
    rospy.init_node("coupling_slider_gui")
    spin_thread = threading.Thread(target=rospy.spin)
    spin_thread.daemon = True
    spin_thread.start()
    CouplingSliderGui().spin()


if __name__ == "__main__":
    main()
