# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Constrained runtime assets for the task-only recovery deployment."""
from pathlib import Path
from urllib.parse import urlparse
import xml.etree.ElementTree as ET


def source_task_directory():
    """Locate the source tree even when loaded from its task-local overlay."""
    candidates = [Path.cwd() / 'src' / 'task']
    candidates.extend(Path(__file__).resolve().parents)
    for candidate in candidates:
        if (candidate.name == 'task' and candidate.parent.name == 'src'
                and (candidate / 'CMakeLists.txt').is_file()
                and (candidate / 'task_runtime' / 'core.py').is_file()):
            return candidate.resolve()
    raise RuntimeError('Launch from car_ws with its source src/task available; '
                       'runtime assets must remain inside src/task')


def confined_path(task_directory, path):
    root = Path(task_directory).resolve(strict=True)
    target = Path(path).resolve()
    if not target.is_relative_to(root) or target == root:
        raise ValueError(f'Output must be inside source task directory: {root}')
    return target


def prepare_world(source, task_directory, output=None):
    """Add state services to a copy; never modify the supplied world.

    Relative resource URIs are rooted at the original file, so relocation does
    not silently change their meaning. Gazebo model/package URIs stay intact.
    """
    source = Path(source).resolve(strict=True)
    target = confined_path(task_directory, output or
                           Path(task_directory) / 'logs/runtime/recovery.world')
    if target == source:
        raise ValueError('World output must differ from the source world')
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    document = ET.parse(source, parser=parser)
    worlds = document.getroot().findall('world')
    if len(worlds) != 1:
        raise ValueError('Recovery launch requires exactly one SDF world')
    world = worlds[0]
    for uri in world.iter('uri'):
        value = (uri.text or '').strip()
        parsed = urlparse(value)
        if value and not parsed.scheme and not Path(value).is_absolute():
            uri.text = (source.parent / value).resolve().as_uri()
        elif value.startswith('file://') and not value.startswith('file:///'):
            uri.text = (source.parent / value[7:]).resolve().as_uri()
    existing = [p for p in world.findall('plugin')
                if p.get('filename', '').endswith('libgazebo_ros_state.so')]
    if existing:
        if len(existing) != 1 or existing[0].findtext('ros/namespace') != '/task/gazebo':
            raise ValueError('Existing Gazebo state plugin has a different namespace')
    else:
        plugin = ET.SubElement(world, 'plugin', name='task_gazebo_state',
                               filename='libgazebo_ros_state.so')
        ros = ET.SubElement(plugin, 'ros')
        ET.SubElement(ros, 'namespace').text = '/task/gazebo'
        ET.SubElement(plugin, 'update_rate').text = '30.0'
    target.parent.mkdir(parents=True, exist_ok=True)
    document.write(target, encoding='utf-8', xml_declaration=True)
    return target
