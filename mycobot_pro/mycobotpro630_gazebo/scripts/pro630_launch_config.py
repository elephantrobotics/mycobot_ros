"""Shared robot descriptions and MoveIt profiles for ROS 1 launchers."""
import xacro
import yaml

from pro630_configuration import joints, project_semantic

PACKAGE = "mycobotpro630_gazebo"


def profile(package_path, with_gripper, gazebo=False, contact=True):
    options = {
        'with_gripper': str(with_gripper).lower(),
        'collision_mesh_dir': 'collision_gazebo' if gazebo else 'collision_exact',
        'gazebo_control': str(gazebo).lower(),
        'gazebo_contact_response': str(contact).lower(),
    }
    return xacro.process_file(str(package_path / 'config/firefighter.urdf.xacro'),
                              mappings=options).toxml()


def read_yaml(package_path, name):
    return yaml.safe_load((package_path / 'config' / name).read_text(encoding='utf-8'))


def descriptions(package_path, with_gripper):
    semantic = (package_path / 'config/firefighter.srdf').read_text(encoding='utf-8')
    planning = read_yaml(package_path, 'joint_limits.yaml')
    planning['joint_limits'] = {name: spec for name, spec in planning['joint_limits'].items()
                                if name in joints(with_gripper)}
    planning.update(read_yaml(package_path, 'pilz_cartesian_limits.yaml'))
    return {
        '/robot_description': profile(package_path, with_gripper),
        '/robot_description_semantic': project_semantic(semantic, with_gripper),
        '/robot_description_kinematics': read_yaml(package_path, 'kinematics.yaml'),
        '/robot_description_planning': planning,
    }


def move_group_params(package_path, with_gripper):
    ompl = read_yaml(package_path, 'ompl_planning.yaml')
    start_adapter = 'pro630_gazebo/UseCurrentStartState'
    checked_adapters = ' '.join((start_adapter, *(
        'default_planner_request_adapters/' + name for name in (
            'AddTimeOptimalParameterization', 'ResolveConstraintFrames', 'FixWorkspaceBounds',
            'FixStartStateBounds', 'FixStartStateCollision', 'FixStartStatePathConstraints'))))
    common = dict(request_adapters=checked_adapters, start_state_max_bounds_error=.1,
                  jiggle_fraction=.05)
    ompl.update(common, planning_plugin='ompl_interface/OMPLPlanner')
    pipelines = {
        'ompl': ompl,
        'chomp': dict(common, planning_plugin='chomp_interface/CHOMPPlanner',
                      enable_failure_recovery=True, ridge_factor=.01),
        'pilz_industrial_motion_planner': dict(
            planning_plugin='pilz_industrial_motion_planner::CommandPlanner',
            request_adapters=start_adapter, default_planner_config='PTP',
            capabilities='pilz_industrial_motion_planner/MoveGroupSequenceAction '
                         'pilz_industrial_motion_planner/MoveGroupSequenceService'),
    }
    control = read_yaml(package_path, 'moveit_controllers.yaml')
    if not with_gripper:
        ompl.pop('gripper', None)
        control['controller_list'] = [entry for entry in control['controller_list']
                                      if entry['name'] != 'pro_gripper_controller']
    params = dict(control, planning_pipelines=pipelines, default_planning_pipeline='ompl')
    return {f'/move_group/{key}': value for key, value in params.items()}


