#!/usr/bin/env python3

"""Deterministically probe the MoveIt collision matrix with random joint states."""

import argparse
import random

import pro630_ros1 as ros
from moveit_msgs.srv import GetStateValidity
from pro630_ros1 import Node


from pro630_configuration import joints, JOINT_LIMITS_RAD, public_profile


def pair(contact):
    return frozenset((contact.contact_body_1, contact.contact_body_2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=630)
    args = parser.parse_args()

    with_gripper = public_profile()
    JOINTS = joints(with_gripper)
    LIMITS = JOINT_LIMITS_RAD[:len(JOINTS)]
    ros.init()
    node = Node("pro630_collision_probe")
    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=10.0):
        raise SystemExit("/check_state_validity is unavailable")

    rng = random.Random(args.seed)
    targets = {
        "link2/link5": frozenset(("link2", "link5")),
        "link2/gripper": None,
    }
    found = {}
    invalid = 0
    for index in range(args.samples):
        positions = [rng.uniform(low, high) for low, high in LIMITS]
        request = GetStateValidity._request_class()
        request.group_name = ""
        request.robot_state.joint_state.name = JOINTS
        request.robot_state.joint_state.position = positions
        future = client.call_async(request)
        ros.spin_until_future_complete(node, future, timeout_sec=2.0)
        response = future.result()
        if response is None:
            raise SystemExit(f"state validity request {index} failed")
        if response.valid:
            continue
        invalid += 1
        pairs = {pair(contact) for contact in response.contacts}
        if targets["link2/link5"] in pairs and "link2/link5" not in found:
            found["link2/link5"] = positions
        if "link2/gripper" not in found:
            for bodies in pairs:
                if "link2" in bodies and any(name.startswith("gripper_") for name in bodies):
                    found["link2/gripper"] = positions
                    break
        if len(found) == len(targets):
            break

    print(f"samples={index + 1} invalid={invalid}")
    for name in targets:
        print(f"{name}: {found.get(name, 'NOT FOUND')}")
    node.destroy_node()
    ros.shutdown()


if __name__ == "__main__":
    main()
