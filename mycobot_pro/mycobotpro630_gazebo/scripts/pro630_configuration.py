"""One installation profile shared by model generation and control processes."""
import xml.etree.ElementTree as ET

from pro630_model import ARM_JOINTS, GRIPPER_JOINT, LIMITS_DEG, JOINT_LIMITS_RAD


def require_flag(value):
    if type(value) is not bool:
        raise ValueError('Missing or invalid with_gripper capability; restart the matching public environment')
    return value


def collision_policy(value):
    if type(value) is not bool:
        raise ValueError('Missing or invalid allow_collision capability; restart the matching public environment')
    return value


def joints(with_gripper):
    return ARM_JOINTS + ([GRIPPER_JOINT] if require_flag(with_gripper) else [])


def controllers(with_gripper):
    return ['arm_controller', 'joint_state_broadcaster'] + (
        ['pro_gripper_controller'] if require_flag(with_gripper) else [])


def geometry_xml(path, with_gripper=True):
    """Keep offline geometry tools compatible with the conditional URDF source."""
    root = ET.parse(path).getroot()
    for parent in list(root.iter()):
        for child in list(parent):
            if child.tag == '{http://www.ros.org/wiki/xacro}if' and child.get('value') == '$(arg with_gripper)':
                index = list(parent).index(child)
                parent.remove(child)
                if require_flag(with_gripper):
                    for item in list(child):
                        parent.insert(index, item)
                        index += 1
    return root


def configure_node(node):
    node.declare_parameter('with_gripper', True)
    node.with_gripper = require_flag(node.get_parameter('with_gripper').value)
    node.declare_parameter('allow_collision', False)
    node.allow_collision = collision_policy(node.get_parameter('allow_collision').value)
    node.command_joints = joints(node.with_gripper)
    node.joint_limits = JOINT_LIMITS_RAD[:len(node.command_joints)]


def project_semantic(text, with_gripper):
    if require_flag(with_gripper):
        return text
    root = ET.fromstring(text)
    links = {'world', 'base', *[f'link{i}' for i in range(1, 7)]}
    names = {'world_to_base', *ARM_JOINTS}
    for child in list(root):
        if (child.tag == 'group' and child.get('name') == 'gripper' or
                child.tag == 'group_state' and child.get('group') == 'gripper' or
                child.tag == 'end_effector' or
                child.tag == 'passive_joint' and child.get('name') not in names or
                child.tag == 'disable_collisions' and
                (child.get('link1') not in links or child.get('link2') not in links)):
            root.remove(child)
        elif child.tag == 'group' and child.get('name') == 'arm':
            for item in list(child):
                if item.tag == 'joint' and item.get('name') not in names:
                    child.remove(item)
                elif item.tag == 'chain':
                    item.set('tip_link', 'link6')
    return ET.tostring(root, encoding='unicode')


def public_profile(timeout=2.0):
    """Auxiliary visualizers inherit the active installation profile."""
    import json
    import rospy
    from std_msgs.msg import String
    try:
        msg = rospy.wait_for_message('/pro630/environment_state', String, timeout=timeout)
    except rospy.ROSException:
        return True
    return require_flag(json.loads(msg.data).get('with_gripper'))
