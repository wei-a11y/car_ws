# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from dataclasses import FrozenInstanceError
import json

import pytest
import yaml

from task_runtime.core import load_mission, MissionEngine, order_dropoffs
from task_runtime.journal import Journal


def config():
    return dict(configured=True, frame='map',
                home=dict(name='home', x=0, y=0, yaw=0),
                pickup=dict(name='pickup', x=1, y=0, yaw=0.5),
                dropoffs=[dict(name='far', x=5, y=0, yaw=1),
                          dict(name='near', x=2, y=0, yaw=-1),
                          dict(name='tie', x=0, y=0, yaw=3)])


def write(tmp_path, data=None):
    path = tmp_path / 'points.yaml'
    path.write_text(yaml.safe_dump(config() if data is None else data))
    return path


def test_snapshot_reload_and_stable_extensible_sort(tmp_path):
    path = write(tmp_path)
    mission = load_mission(path, 'map')
    assert [p.name for p in order_dropoffs(mission)] == ['near', 'tie', 'far']
    descending = order_dropoffs(mission, lambda origin, p: -p.x)
    assert [p.name for p in descending] == ['far', 'near', 'tie']
    with pytest.raises(FrozenInstanceError):
        mission.home.x = 10
    changed = config()
    changed['home']['x'] = 10
    write(tmp_path, changed)
    assert mission.home.x == 0
    assert load_mission(path, 'map').home.x == 10


@pytest.mark.parametrize('change,match', [
    (lambda c: c.update(configured=False), 'configured'),
    (lambda c: c.update(frame='odom'), 'frame'),
    (lambda c: c.update(dropoffs=[]), 'dropoffs'),
    (lambda c: c['home'].update(name='pickup'), 'unique'),
    (lambda c: c['home'].update(x=float('nan')), 'home.x'),
    (lambda c: c['pickup'].update(y=True), 'pickup.y'),
    (lambda c: c['dropoffs'][0].pop('yaw'), 'dropoffs\\[0\\].yaw'),
])
def test_validation(tmp_path, change, match):
    data = config()
    change(data)
    with pytest.raises(ValueError, match=match):
        load_mission(write(tmp_path, data), 'map')


def test_invalid_yaml_and_missing_file(tmp_path):
    with pytest.raises(ValueError, match='Cannot read'):
        load_mission(tmp_path / 'missing.yaml', 'map')
    path = tmp_path / 'invalid.yaml'
    path.write_text('home: [')
    with pytest.raises(ValueError, match='Cannot read'):
        load_mission(path, 'map')


def test_full_round_timing_event_dedup_and_new_round(tmp_path):
    mission = load_mission(write(tmp_path), 'map')
    engine = MissionEngine()
    assert engine.start(mission, 'round1') == [('GOAL', mission.pickup)]
    with pytest.raises(ValueError, match='already active'):
        engine.start(mission, 'double')
    assert engine.reached(10)[0][0] == 'PICKUP_STARTED'
    assert engine.reached(10.1) == []
    assert engine.tick(12.999) == []
    effects = engine.tick(13)
    assert [e for e, _ in effects] == ['PICKUP_DONE', 'DELIVERY_ORDER', 'GOAL']
    assert engine.tick(13.1) == []
    for i, point in enumerate(engine.order):
        assert engine.target == point
        assert engine.reached(20 + i * 10)[0][0] == 'DROPOFF_STARTED'
        assert engine.tick(22 + i * 10) == []
        assert engine.tick(23 + i * 10)[0] == ('DROPOFF_DONE', point)
    assert engine.state == 'NAV_HOME'
    assert engine.reached(60) == [('TASK_DONE', mission.home)]
    assert engine.state == 'IDLE'
    assert engine.reached(60.1) == []
    engine.start(mission, 'round2')
    assert engine.state == 'NAV_PICKUP'
    engine.fail()
    assert engine.tick(100) == []
    assert engine.reached(100) == []


def test_journal_rotation_and_nonfinite(tmp_path):
    journal = Journal(tmp_path, max_bytes=100, backups=2)
    for i in range(20):
        journal.write({'sequence': i, 'range': float('inf')})
    paths = list(tmp_path.glob('tasks.jsonl*'))
    assert len(paths) == 3
    assert all(p.stat().st_size <= 100 for p in paths)
    assert json.loads(journal.path.read_text().splitlines()[-1])['sequence'] == 19


def test_journal_creation_and_write_errors_propagate(tmp_path):
    file = tmp_path / 'not_directory'
    file.write_text('x')
    with pytest.raises(OSError):
        Journal(file)
    journal = Journal(tmp_path / 'logs')
    journal.path.unlink()
    journal.path.mkdir()
    with pytest.raises(OSError):
        journal.write({'event': 'fail'})
