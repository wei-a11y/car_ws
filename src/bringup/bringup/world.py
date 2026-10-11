# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Create a temporary ODE world using the Gazebo arm physics settings."""
from pathlib import Path
import math
from numbers import Real
import tempfile
import xml.etree.ElementTree as ET

import yaml


def _numeric_setting(settings, name, allow_zero=False):
    """Read a finite numeric setting with the required positive range."""
    value = settings[name]
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if value < 0 or (value == 0 and not allow_zero):
        bound = 'nonnegative' if allow_zero else 'positive'
        raise ValueError(f'{name} must be {bound}')
    return float(value)


def _single_child(parent, name):
    """Return or create one child, rejecting ambiguous duplicate elements."""
    children = parent.findall(name)
    if len(children) > 1:
        raise ValueError(f'Expected at most one {parent.tag}/{name} element')
    return children[0] if children else ET.SubElement(parent, name)


def gazebo_world_file(world_path, physics_params_file):
    """Return a temporary world with validated ODE settings for caller cleanup."""
    settings = yaml.safe_load(Path(physics_params_file).read_text(encoding='utf-8'))
    names = {'solver_type', 'cfm', 'max_step_size', 'real_time_update_rate'}
    if not isinstance(settings, dict) or set(settings) != names:
        raise ValueError('Physics YAML must specify exactly: ' + ', '.join(sorted(names)))
    solver_type = settings['solver_type']
    if solver_type not in ('world', 'quick'):
        raise ValueError('solver_type must be world or quick')
    cfm = _numeric_setting(settings, 'cfm', allow_zero=True)
    step = _numeric_setting(settings, 'max_step_size')
    update_rate = _numeric_setting(settings, 'real_time_update_rate')

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True, insert_pis=True))
    tree = ET.parse(Path(world_path), parser=parser)
    root = tree.getroot()
    worlds = root.findall('world')
    if root.tag != 'sdf' or len(worlds) != 1:
        raise ValueError('Expected an SDF document containing exactly one world')
    physics_elements = worlds[0].findall('physics')
    if len(physics_elements) != 1 or physics_elements[0].get('type') != 'ode':
        raise ValueError('Expected exactly one ODE physics element in the world')
    physics = physics_elements[0]
    _single_child(physics, 'max_step_size').text = str(step)
    _single_child(physics, 'real_time_update_rate').text = str(update_rate)
    ode = _single_child(physics, 'ode')
    _single_child(_single_child(ode, 'solver'), 'type').text = solver_type
    _single_child(_single_child(ode, 'constraints'), 'cfm').text = str(cfm)

    with tempfile.NamedTemporaryFile(
            mode='wb', prefix='car_ws_gazebo_world_', suffix='.world', delete=False) as output:
        path = Path(output.name)
        try:
            tree.write(output, encoding='utf-8', xml_declaration=True)
        except Exception:
            path.unlink(missing_ok=True)
            raise
    return path
