#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Compatibility alias for sync_plan.py.

Prefer:
  rosrun mycobot_pro450_moveit sync_plan.py

sync_plan.py auto-detects gripper_controller in /joint_states and enables
F100 gripper sync when present. This script remains for older docs/commands.
"""

from pathlib import Path
import runpy
import sys


def main():
    print(
        "NOTE: sync_plan_force_gripper.py is a compatibility alias; "
        "prefer rosrun mycobot_pro450_moveit sync_plan.py "
        "(auto-detects F100 via gripper_controller)."
    )
    target = Path(__file__).with_name("sync_plan.py")
    if not target.is_file():
        sys.stderr.write("Unable to locate sync_plan.py next to {}\n".format(__file__))
        sys.exit(1)
    runpy.run_path(str(target), run_name="__main__")


if __name__ == "__main__":
    main()
