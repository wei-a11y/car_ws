# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Adapt the existing MoveIt description to Gazebo's single controller manager."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml


def gazebo_moveit_config(arm_params_file=None):
    """Keep the model geometry and merge mock arm interfaces into Gazebo hardware."""
    # Keep the base-only launch independent of xacro/MoveIt initialization.
    from moveit_configs_utils import MoveItConfigsBuilder

    description_share = Path(get_package_share_directory('car_description'))
    bringup_share = Path(get_package_share_directory('bringup'))
    arm_params = Path(arm_params_file or bringup_share / 'config/gazebo_arm.yaml')
    settings = yaml.safe_load(arm_params.read_text(encoding='utf-8'))['joint_settings']
    config = MoveItConfigsBuilder(
        'car_urdf', package_name='arm_moveit_config').to_moveit_configs()
    robot = ET.fromstring(config.robot_description['robot_description'])
    systems = robot.findall('ros2_control')
    gazebo = [system for system in systems if system.findtext('hardware/plugin') ==
              'gazebo_ros2_control/GazeboSystem']
    mock = [system for system in systems if system.findtext('hardware/plugin') ==
            'mock_components/GenericSystem']
    if len(systems) != 2 or len(gazebo) != 1 or len(mock) != 1:
        raise RuntimeError('Expected one Gazebo base system and one mock arm system')
    gazebo_system, arm_system = gazebo[0], mock[0]
    registered = {joint.get('name') for joint in gazebo_system.findall('joint')}
    if set(settings) != {joint.get('name') for joint in arm_system.findall('joint')}:
        raise RuntimeError('Simulation joint settings must match all arm hardware joints')
    for joint in arm_system.findall('joint'):
        name = joint.get('name')
        if name in registered:
            raise RuntimeError(f'Duplicate hardware joint: {name}')
        registered.add(name)
        values = settings[name]
        for key in ['velocity', 'acceleration', 'effort', 'kp', 'integral_limit']:
            if not math.isfinite(values[key]) or values[key] <= 0:
                raise RuntimeError(f'{name}.{key} must be finite and positive')
        for key in ['ki', 'kd', 'damping']:
            if not math.isfinite(values[key]) or values[key] < 0:
                raise RuntimeError(f'{name}.{key} must be finite and nonnegative')
        limit = robot.find(f"joint[@name='{name}']/limit")
        initial = values['initial_position']
        if not math.isfinite(initial) or not float(limit.get('lower')) <= initial <= float(
                limit.get('upper')):
            raise RuntimeError(f'{name} simulation initial position is out of bounds')
        limit.set('effort', str(values['effort']))
        limit.set('velocity', str(values['velocity']))
        physical_joint = robot.find(f"joint[@name='{name}']")
        dynamics = physical_joint.find('dynamics')
        if dynamics is None:
            dynamics = ET.SubElement(physical_joint, 'dynamics')
        dynamics.set('damping', str(values['damping']))
        # Integrate the passive joint damping implicitly in ODE. The hardware
        # PID derivative term remains separate from this physical damping.
        extension = robot.find(f"gazebo[@reference='{name}']")
        if extension is None:
            extension = ET.SubElement(robot, 'gazebo', reference=name)
        implicit = extension.find('implicitSpringDamper')
        if implicit is None:
            implicit = ET.SubElement(extension, 'implicitSpringDamper')
        implicit.text = 'true'
        command = joint.find("command_interface[@name='position']")
        command.set('name', 'position_pid')
        joint.find("state_interface[@name='position']/param[@name='initial_value']").text = str(
            initial)
        for key, value in {
            'pos_kp': values['kp'], 'pos_ki': values['ki'], 'pos_kd': values['kd'],
            'pos_max_integral_error': values['integral_limit'],
            'pos_min_integral_error': -values['integral_limit'],
            'pos_antiwindup': 'true',
        }.items():
            ET.SubElement(joint, 'param', name=key).text = str(value)
        config.joint_limits['robot_description_planning']['joint_limits'][name].update({
            'has_velocity_limits': True, 'max_velocity': values['velocity'],
            'has_acceleration_limits': True, 'max_acceleration': values['acceleration'],
        })
        gazebo_system.append(joint)
    robot.remove(arm_system)

    plugins = robot.findall("gazebo/plugin[@filename='libgazebo_ros2_control.so']")
    if len(plugins) != 1:
        raise RuntimeError('Expected one gazebo_ros2_control plugin')
    # The first YAML retains the base's odometry, command remaps and sim clock.
    # The second registers the arm/gripper trajectory controllers in the same manager.
    ET.SubElement(plugins[0], 'parameters').text = str(
        config.package_path / 'config' / 'ros2_controllers.yaml')
    ET.SubElement(plugins[0], 'parameters').text = str(
        bringup_share / 'config/gazebo_arm_controllers.yaml')
    description = ET.tostring(robot, encoding='unicode')
    description = description.replace('$(find car_description)', str(description_share))
    description = description.replace(
        'package://car_description/', f'file://{description_share}/')
    config.robot_description['robot_description'] = description
    return config
