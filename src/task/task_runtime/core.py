# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""ROS-independent immutable configuration, scoring and mission transitions."""
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Callable

import yaml


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Mission:
    frame: str
    home: Waypoint
    pickup: Waypoint
    dropoffs: tuple


def load_mission(filename: str, expected_frame: str) -> Mission:
    try:
        data = yaml.safe_load(Path(filename).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'Cannot read mission YAML {filename}: {error}') from error
    if not isinstance(data, dict):
        raise ValueError('mission YAML must be a mapping')
    if data.get('configured') is not True:
        raise ValueError('Set configured: true only after replacing example map coordinates')
    if data.get('frame') != expected_frame:
        raise ValueError(f'frame must equal verified planning frame {expected_frame!r}')

    def point(value, field):
        if not isinstance(value, dict):
            raise ValueError(f'{field} must contain name, x, y, yaw')
        name = value.get('name')
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f'{field}.name must be a nonempty string')
        numbers = []
        for key in ('x', 'y', 'yaw'):
            number = value.get(key)
            if isinstance(number, bool) or not isinstance(number, (int, float)):
                raise ValueError(f'{field}.{key} must be numeric (metres/radians)')
            if not math.isfinite(number):
                raise ValueError(f'{field}.{key} must be finite')
            numbers.append(float(number))
        return Waypoint(name, *numbers)

    home = point(data.get('home'), 'home')
    pickup = point(data.get('pickup'), 'pickup')
    raw = data.get('dropoffs')
    if not isinstance(raw, list) or not raw:
        raise ValueError('dropoffs must contain at least one waypoint')
    drops = tuple(point(value, f'dropoffs[{i}]') for i, value in enumerate(raw))
    names = [p.name for p in (home, pickup, *drops)]
    if len(set(names)) != len(names):
        raise ValueError('Waypoint names must be unique across home, pickup and dropoffs')
    return Mission(expected_frame, home, pickup, drops)


def distance_cost(origin: Waypoint, target: Waypoint) -> float:
    return math.hypot(origin.x - target.x, origin.y - target.y)


def order_dropoffs(mission: Mission, scorer: Callable = distance_cost) -> tuple:
    """Stable, one-time sorting relative to pickup; scorer is the extension point."""
    scored = [(scorer(mission.pickup, p), p) for p in mission.dropoffs]
    if any(not math.isfinite(cost) for cost, _ in scored):
        raise ValueError('Sorting cost must be finite')
    return tuple(p for _, p in sorted(scored, key=lambda item: item[0]))


class MissionEngine:
    """Caller supplies ROS time; all actions are returned, never executed here."""

    def __init__(self, pickup_delay=3.0, dropoff_delay=3.0, scorer=distance_cost):
        self.pickup_delay = pickup_delay
        self.dropoff_delay = dropoff_delay
        self.scorer = scorer
        self.state = 'IDLE'
        self.mission = None
        self.target = None
        self.task_id = None
        self.order = ()
        self.index = 0
        self.deadline = None

    def start(self, mission, task_id):
        if self.state not in ('IDLE', 'FAILED'):
            raise ValueError(f'Task already active: {self.state}')
        self.mission, self.task_id = mission, task_id
        self.order, self.index, self.deadline = (), 0, None
        self.target, self.state = mission.pickup, 'NAV_PICKUP'
        return [('GOAL', self.target)]

    def reached(self, now):
        if self.state == 'NAV_PICKUP':
            self.state, self.deadline = 'PICKING', now + self.pickup_delay
            return [('PICKUP_STARTED', self.target)]
        if self.state == 'NAV_DROPOFF':
            self.state, self.deadline = 'DROPPING', now + self.dropoff_delay
            return [('DROPOFF_STARTED', self.target)]
        if self.state == 'NAV_HOME':
            self.state = 'IDLE'
            return [('TASK_DONE', self.target)]
        return []

    def tick(self, now):
        if self.deadline is None or now < self.deadline:
            return []
        self.deadline = None
        if self.state == 'PICKING':
            self.order = order_dropoffs(self.mission, self.scorer)
            effects = [('PICKUP_DONE', self.target), ('DELIVERY_ORDER', self.order)]
            self.target, self.state = self.order[0], 'NAV_DROPOFF'
        elif self.state == 'DROPPING':
            effects = [('DROPOFF_DONE', self.target)]
            self.index += 1
            if self.index < len(self.order):
                self.target, self.state = self.order[self.index], 'NAV_DROPOFF'
            else:
                self.target, self.state = self.mission.home, 'NAV_HOME'
        else:
            return []
        return effects + [('GOAL', self.target)]

    def fail(self):
        self.state, self.deadline = 'FAILED', None
