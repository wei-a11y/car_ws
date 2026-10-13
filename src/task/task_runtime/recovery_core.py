# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""ROS independent geometry, observable corridors and complete recovery candidates.

The binding is deliberately thin: all map inflation, A* and line checks are the
existing ``plan`` C++ implementation. Costs describe the virtual manipulation
backend, not measured mechanical work or hardware safety limits.
"""
from dataclasses import dataclass, field, replace
import math
from typing import Optional


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose3:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.z, self.yaw)):
            raise ValueError('Pose coordinates and yaw must be finite')


@dataclass(frozen=True)
class Obstacle:
    object_id: str
    label: str
    pose: Pose3
    size: tuple
    confidence: float
    stamp_ns: int
    frame: str = 'map'

    def __post_init__(self):
        if not self.object_id or not self.label or not self.frame:
            raise ValueError('Object identity, label and frame are required')
        if len(self.size) != 3 or not all(math.isfinite(v) and v > 0 for v in self.size):
            raise ValueError('Obstacle size requires three positive finite dimensions')
        if not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError('Confidence must be finite within [0, 1]')
        if not isinstance(self.stamp_ns, int) or self.stamp_ns < 0:
            raise ValueError('Observation stamp must be nonnegative integer nanoseconds')
        object.__setattr__(self, 'size', tuple(self.size))


@dataclass(frozen=True)
class CorridorResult:
    state: str
    clear_distance: float
    blocking_points: tuple = ()


@dataclass(frozen=True)
class Candidate:
    kind: str
    object_id: str
    action_pose: Optional[Pose3] = None
    pass_pose: Optional[Pose3] = None
    restore_pose: Optional[Pose3] = None
    temporary_pose: Optional[Pose3] = None
    original_pose: Optional[Pose3] = None
    route: tuple = ()
    time_cost: float = 0.0
    work_cost: float = 0.0
    score: float = 0.0
    # Explicit legs and rollback remain available to execution and audit.
    routes: dict = field(default_factory=dict, compare=False)
    rollback_route: tuple = ()
    extra_radius: float = 0.0


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def route_length(route):
    return sum(distance(a, b) for a, b in zip(route, route[1:]))


def _pose(point):
    if isinstance(point, Pose3):
        return point
    if hasattr(point, 'pose'):
        point = point.pose
    if hasattr(point, 'position'):
        q = point.orientation
        return Pose3(point.position.x, point.position.y, point.position.z,
                     math.atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z)))
    if hasattr(point, 'x'):
        return Pose3(point.x, point.y, getattr(point, 'z', 0.0),
                     getattr(point, 'yaw', 0.0))
    return Pose3(*point)


def split_path(points, epsilon=1e-7):
    """Return straight segments; remove duplicates, never smooth a corner.

    A singleton remains a singleton so a final orientation-only command is not
    discarded. Intermediate segment final yaws follow their own tangent; only
    the last segment retains the caller's final goal yaw.
    """
    unique = []
    for point in points:
        p = _pose(point)
        if unique and distance(p, unique[-1]) <= epsilon:
            unique[-1] = p
        else:
            unique.append(p)
    if len(unique) <= 1:
        return [unique] if unique else []
    vertices = [unique[0]]
    for i in range(1, len(unique) - 1):
        a, b, c = vertices[-1], unique[i], unique[i + 1]
        u, v = (b.x - a.x, b.y - a.y), (c.x - b.x, c.y - b.y)
        lengths = math.hypot(*u) * math.hypot(*v)
        if abs(u[0] * v[1] - u[1] * v[0]) > epsilon * lengths or (
                u[0] * v[0] + u[1] * v[1] <= 0):
            vertices.append(b)
    vertices.append(unique[-1])
    result = []
    for i, (a, b) in enumerate(zip(vertices, vertices[1:])):
        yaw = math.atan2(b.y - a.y, b.x - a.x)
        end_yaw = b.yaw if i == len(vertices) - 2 else yaw
        result.append([replace(a, yaw=yaw), replace(b, yaw=end_yaw)])
    return result


class ScanView:
    """One planar scan and the laser pose in map at that scan's timestamp.

    A finite in-range return is evidence only before its measured surface. NaN,
    clipped ranges and gaps have no free-space evidence. Positive infinity can
    explicitly use Gazebo's no-return convention, only inside range_max. The caller
    enforces scan freshness and TF timestamp matching before construction.
    """

    def __init__(self, scan, origin, allow_positive_infinity=False):
        self.scan, self.origin = scan, _pose(origin)
        self.allow_positive_infinity = allow_positive_infinity
        self.ranges = tuple(scan.ranges)
        self.angle_min = float(scan.angle_min)
        self.increment = float(scan.angle_increment)
        self.range_min, self.range_max = float(scan.range_min), float(scan.range_max)
        if (not self.ranges or not all(math.isfinite(v) for v in
                (self.angle_min, self.increment, self.range_min, self.range_max)) or
                self.increment == 0 or self.range_min < 0 or self.range_max <= self.range_min):
            raise ValueError('Invalid scan angular/range geometry')
        stamp = getattr(getattr(scan, 'header', None), 'stamp', None)
        self.stamp_ns = (int(stamp.sec) * 1000000000 + int(stamp.nanosec)) if stamp else 0

    def _ray_indices(self, angle, radius_angle=0.0):
        span = (len(self.ranges) - 1) * self.increment
        midpoint = self.angle_min + span / 2
        angle += round((midpoint - angle) / (2 * math.pi)) * 2 * math.pi
        coordinate = (angle - self.angle_min) / self.increment
        spread = abs(radius_angle / self.increment)
        lo, hi = math.floor(coordinate - spread), math.ceil(coordinate + spread)
        if lo < 0 or hi >= len(self.ranges):
            return ()
        return range(lo, hi + 1)

    def classify(self, point, padding=0.0):
        point = _pose(point)
        if not math.isfinite(padding) or padding < 0:
            raise ValueError('Scan padding must be finite and nonnegative')
        d = distance(point, self.origin)
        if d - padding <= self.range_min or d + padding >= self.range_max:
            return 'unknown'
        angle = math.atan2(point.y - self.origin.y,
                           point.x - self.origin.x) - self.origin.yaw
        rays = self._ray_indices(angle, math.asin(min(1.0, padding / d)))
        if not rays:
            return 'unknown'
        unknown = False
        for index in rays:
            value = self.ranges[index]
            if self.allow_positive_infinity and value == math.inf:
                continue
            if not math.isfinite(value) or not self.range_min < value < self.range_max:
                unknown = True
                continue
            # Half the beam spacing is included in the surface uncertainty.
            tolerance = padding + d * abs(self.increment) / 2 + 1e-6
            if abs(value - d) <= tolerance:
                return 'blocked'
            if value < d - tolerance:
                unknown = True  # behind a measured surface is occluded
        return 'unknown' if unknown else 'clear'

    def hits(self):
        result = []
        for i, value in enumerate(self.ranges):
            if math.isfinite(value) and self.range_min < value < self.range_max:
                angle = self.origin.yaw + self.angle_min + i * self.increment
                result.append(Pose3(self.origin.x + value * math.cos(angle),
                                    self.origin.y + value * math.sin(angle), self.origin.z))
        return result

    def visible(self, obstacle):
        """A target must have a return on its front face, not just be in range."""
        d = distance(obstacle.pose, self.origin)
        radius = math.hypot(*obstacle.size[:2]) / 2
        if d <= radius or d - radius > self.range_max:
            return False
        angle = math.atan2(obstacle.pose.y - self.origin.y,
                           obstacle.pose.x - self.origin.x) - self.origin.yaw
        rays = self._ray_indices(angle, math.asin(min(1.0, radius / d)))
        if not rays:
            return False
        seen = False
        for i in rays:
            value = self.ranges[i]
            if not math.isfinite(value) or not self.range_min < value < self.range_max:
                continue
            if value < d - radius - 0.02:
                continue
            if value <= d + radius + 0.02:
                seen = True
        return seen

    def inspect_segment(self, start, end, radius, spacing=0.1, stop_margin=0.25,
                        self_clear_radius=0.0):
        """Observe a swept corridor and the braking distance before its first gap.

        ``self_clear_radius`` may only represent a known collision-free disk
        already occupied by the stopped robot at ``start``. It defaults to zero,
        does not clear any scan hit, and never extends to future robot positions.
        """
        start, end = _pose(start), _pose(end)
        if any(not math.isfinite(v) or v < 0 for v in
               (radius, stop_margin, self_clear_radius)) or spacing <= 0:
            raise ValueError('Invalid corridor geometry')
        length = distance(start, end)
        theta = math.atan2(end.y - start.y, end.x - start.x)
        c, s = math.cos(theta), math.sin(theta)
        blocking = []
        first_hit = math.inf
        for p in self.hits():
            dx, dy = p.x - start.x, p.y - start.y
            along, across = dx * c + dy * s, -dx * s + dy * c
            endpoint_distance = along - min(length, max(0.0, along))
            if endpoint_distance**2 + across**2 <= radius**2:
                blocking.append(p)
                first_hit = min(first_hit, max(0.0, along - radius))
        count = max(1, math.ceil(length / spacing))
        offsets = max(1, math.ceil(2 * radius / spacing))
        clear_distance, first_unknown = 0.0, math.inf
        for i in range(count + 1):
            along = min(length, length * i / count)
            good = True
            # Include front cap: the base reference cannot approach a surface
            # closer than its complete circular envelope.
            for cross_i in range(offsets + 1):
                across = -radius + 2 * radius * cross_i / offsets
                for longitudinal in (0.0, math.sqrt(max(0.0, radius**2 - across**2))):
                    p = Pose3(start.x + (along + longitudinal) * c - across * s,
                              start.y + (along + longitudinal) * s + across * c)
                    if self_clear_radius and distance(p, start) <= self_clear_radius:
                        continue
                    if self.classify(p) != 'clear':
                        good = False
            if not good:
                first_unknown = along
                break
            clear_distance = along
        if first_unknown == math.inf and first_hit == math.inf:
            return CorridorResult('clear', length)
        limit = min(first_unknown, first_hit, clear_distance)
        return CorridorResult('blocked' if blocking else 'unknown',
                              max(0.0, limit - stop_margin), tuple(blocking))


PLANNING_DEFAULTS = {
    'occupied_threshold': 65, 'unknown_is_obstacle': True,
    'robot_radius': 0.355, 'safety_margin': 0.05, 'inflate_map_boundary': True,
    'max_cells': 4000000, 'enable_shortcut': True, 'resample_spacing': 0.1,
    'shortcut_max_lookahead': 200, 'max_output_points': 100000,
}
GEOMETRY_DEFAULTS = {
    'arm_reach': 1.3, 'action_distance': 0.85, 'lift_height': 0.35,
    'carry_height': 1.2, 'pass_distance': 0.85, 'temporary_offsets': [0.75, -0.75],
    'action_angles': [0.0, -0.4, 0.4], 'grasp_clearance': 0.08,
    'manipulation_step': 0.08, 'base_position_tolerance': 0.03,
}
COST_DEFAULTS = {
    'time_weight': 1.0, 'work_weight': 1.0, 'time_reference': 60.0,
    'work_reference': 100.0, 'linear_speed': 0.2, 'angular_speed': 0.4,
    'grasp_seconds': 8.0, 'place_seconds': 5.0, 'rolling_force': 8.0,
    'carry_force_per_kg': 1.0, 'gravity': 9.81,
}
CLASS_DEFAULTS = {
    'bag': {'movable': True, 'carry': True, 'mass': 0.5},
    'stool': {'movable': False, 'carry': False, 'mass': 4.0},
    'box': {'movable': True, 'carry': False, 'mass': 1.0},
}


class GridModel:
    """Immutable source-map values with removable dynamic overlays only."""

    def __init__(self, geometry, raw, config=None, frame='map'):
        self.geometry = dict(geometry)
        self.raw = tuple(raw)
        self.config = {**PLANNING_DEFAULTS, **(config or {})}
        if self.config.get('unknown_is_obstacle', True) is not True:
            raise ValueError('Recovery requires unknown_is_obstacle=true')
        self.frame = frame
        self._cache = {}
        if not frame:
            raise ValueError('Grid frame must not be empty')
        # Native validation covers integer geometry, occupancy range and limits.
        self._native(self.raw, 0.0)

    @classmethod
    def from_message(cls, message, config=None):
        origin = message.info.origin
        q = origin.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if (not math.isfinite(norm) or abs(norm - 1) > 1e-6 or
                abs(q.x) > 1e-6 or abs(q.y) > 1e-6):
            raise ValueError('Grid origin must have a planar unit quaternion')
        geometry = dict(width=message.info.width, height=message.info.height,
                        resolution=message.info.resolution, origin_x=origin.position.x,
                        origin_y=origin.position.y, origin_yaw=2 * math.atan2(q.z, q.w))
        return cls(geometry, message.data, config, message.header.frame_id)

    def _native(self, raw, extra_radius):
        try:
            import _planning_eval
        except ImportError as error:
            raise RuntimeError('Build task and source its install: _planning_eval is required') from error
        key = (tuple(raw), extra_radius)
        if key not in self._cache:
            if len(self._cache) >= 16:
                self._cache.pop(next(iter(self._cache)))
            self._cache[key] = _planning_eval.Grid(self.geometry, list(raw),
                                                  self.config, extra_radius)
        return self._cache[key]

    def _cell(self, x, y):
        g = self.geometry
        dx, dy = x - g.get('origin_x', 0), y - g.get('origin_y', 0)
        yaw = g.get('origin_yaw', 0)
        c, s, res = math.cos(yaw), math.sin(yaw), g['resolution']
        return math.floor((c * dx + s * dy) / res), math.floor((-s * dx + c * dy) / res)

    def _world(self, x, y):
        g = self.geometry
        u, v = (x + 0.5) * g['resolution'], (y + 0.5) * g['resolution']
        c, s = math.cos(g.get('origin_yaw', 0)), math.sin(g.get('origin_yaw', 0))
        return (g.get('origin_x', 0) + c * u - s * v,
                g.get('origin_y', 0) + s * u + c * v)

    def snapped_pose(self, pose):
        x, y = self._world(*self._cell(pose.x, pose.y))
        return replace(pose, x=x, y=y)

    def overlay(self, obstacles=(), hits=(), exclude_ids=()):
        cells = list(self.raw)
        g, exclude_ids = self.geometry, set(exclude_ids)
        width, height = g['width'], g['height']
        half_cell = g['resolution'] / math.sqrt(2)
        for obj in obstacles:
            if obj.object_id in exclude_ids:
                continue
            if obj.frame != self.frame:
                raise ValueError('Obstacle and grid frames must match')
            radius = math.hypot(*obj.size[:2]) / 2 + half_cell
            cx, cy = self._cell(obj.pose.x, obj.pose.y)
            n = math.ceil(radius / g['resolution']) + 1
            c, s = math.cos(obj.pose.yaw), math.sin(obj.pose.yaw)
            for y in range(max(0, cy - n), min(height, cy + n + 1)):
                for x in range(max(0, cx - n), min(width, cx + n + 1)):
                    wx, wy = self._world(x, y)
                    dx, dy = wx - obj.pose.x, wy - obj.pose.y
                    if (abs(c * dx + s * dy) <= obj.size[0] / 2 + half_cell and
                            abs(-s * dx + c * dy) <= obj.size[1] / 2 + half_cell):
                        cells[y * width + x] = 100
        for hit in hits:
            hit = _pose(hit)
            x, y = self._cell(hit.x, hit.y)
            if 0 <= x < width and 0 <= y < height:
                cells[y * width + x] = 100
        return cells

    def route(self, start, goal, obstacles=(), hits=(), exclude_ids=(), extra_radius=0.0):
        start, goal = _pose(start), _pose(goal)
        raw = self.overlay(obstacles, hits, exclude_ids)
        result = self._native(raw, extra_radius).route(start.x, start.y, goal.x, goal.y, goal.yaw)
        if result['status'] != 'SUCCESS':
            return None
        return [Pose3(x, y, 0.0, yaw) for x, y, yaw in result['path']]

    def with_unclassified_hits(self, hits, known_obstacles):
        """Keep anonymous returns; known bodies remain removable object overlays.

        Masking their scan cells must never clear a static occupied/unknown cell.
        Call from the evaluation worker because map inflation can be expensive.
        """
        classified = self.overlay(known_obstacles)
        width, height = self.geometry['width'], self.geometry['height']
        anonymous = []
        for point in hits:
            x, y = self._cell(*point[:2])
            if 0 <= x < width and 0 <= y < height and classified[y * width + x] != 100:
                anonymous.append(point)
        return GridModel(self.geometry, self.overlay(hits=anonymous), self.config, self.frame)

    def free(self, pose, obstacles=(), hits=(), exclude_ids=(), extra_radius=0.0):
        pose = _pose(pose)
        return self._native(self.overlay(obstacles, hits, exclude_ids), extra_radius).free(
            pose.x, pose.y)

    def line_clear(self, start, end, obstacles=(), exclude_ids=(), extra_radius=0.0):
        return self._native(self.overlay(obstacles, (), exclude_ids), extra_radius).line_clear(
            start.x, start.y, end.x, end.y)

    def placement_free(self, pose, size, obstacles=(), exclude_ids=(), clearance=0.0):
        """Object footprint in raw known-free map (not the robot inflation)."""
        environment = self.overlay(obstacles, exclude_ids=exclude_ids)
        return self._placement_raw(pose, size, environment, clearance)

    def _placement_raw(self, pose, size, environment, clearance):
        g = self.geometry
        radius = math.hypot(*size[:2]) / 2 + clearance
        # Reject outside map rather than silently clipping the footprint.
        for dx, dy in ((-radius, -radius), (-radius, radius),
                       (radius, -radius), (radius, radius)):
            x, y = self._cell(pose.x + dx, pose.y + dy)
            if not 0 <= x < g['width'] or not 0 <= y < g['height']:
                return False
        cx, cy = self._cell(pose.x, pose.y)
        n = math.ceil((radius + g['resolution']) / g['resolution'])
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        half_cell = g['resolution'] / math.sqrt(2)
        for y in range(max(0, cy - n), min(g['height'], cy + n + 1)):
            for x in range(max(0, cx - n), min(g['width'], cx + n + 1)):
                wx, wy = self._world(x, y)
                dx, dy = wx - pose.x, wy - pose.y
                if (abs(c * dx + s * dy) <= size[0] / 2 + half_cell + clearance and
                        abs(-s * dx + c * dy) <= size[1] / 2 + half_cell + clearance):
                    v = environment[y * g['width'] + x]
                    if v == -1 or v >= self.config['occupied_threshold']:
                        return False
        return True


class RecoveryPlanner:
    """Enumerate whole episodes, reject infeasible restoration, then compare cost.

    This plans in the current static/dynamic map snapshot. It does not certify
    unobserved future space: the execution gate must rescan before every leg,
    after lifting, and after each placement. No candidate moves two objects.
    """

    def __init__(self, config, grid):
        self.config, self.grid = config, grid
        self.geometry = {**GEOMETRY_DEFAULTS, **config.get('geometry', {})}
        self.cost = {**COST_DEFAULTS, **config.get('cost', {})}
        self.classes = config.get('classes', CLASS_DEFAULTS)
        self.perception = {'min_confidence': 0.8, 'max_observation_age_sec': 1.0,
                           **config.get('perception', {})}
        for group in (self.cost, {k: v for k, v in self.geometry.items()
                                 if not isinstance(v, (list, tuple))}):
            for key, value in group.items():
                if not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
                    raise ValueError(f'{key} must be a finite nonnegative number')
        for key in ('time_reference', 'work_reference', 'linear_speed', 'angular_speed'):
            if self.cost[key] <= 0:
                raise ValueError(f'{key} must be positive')
        if self.cost['time_weight'] + self.cost['work_weight'] <= 0:
            raise ValueError('At least one cost weight must be positive')
        for key in ('arm_reach', 'action_distance', 'pass_distance', 'manipulation_step'):
            if self.geometry[key] <= 0:
                raise ValueError(f'{key} must be positive')
        for key in ('action_angles', 'temporary_offsets'):
            if not self.geometry[key] or not all(math.isfinite(v) for v in self.geometry[key]):
                raise ValueError(f'{key} requires finite candidates')
        if (not 0 <= self.perception['min_confidence'] <= 1 or
                not math.isfinite(self.perception['max_observation_age_sec']) or
                self.perception['max_observation_age_sec'] <= 0):
            raise ValueError('Invalid observation confidence/age policy')

    def _turns(self, routes, initial_yaw):
        turns, yaw = 0.0, initial_yaw
        for route in routes:
            if not route:
                continue
            for a, b in zip(route, route[1:]):
                if distance(a, b) > 1e-8:
                    heading = math.atan2(b.y - a.y, b.x - a.x)
                    turns += abs(wrap(heading - yaw))
                    yaw = heading
            turns += abs(wrap(route[-1].yaw - yaw))
            yaw = route[-1].yaw
        return turns

    def _cost(self, kind, routes, base, mass=0.0, carried_distance=0.0):
        c = self.cost
        length = sum(route_length(route) for route in routes)
        grasps, places = {'detour': (0, 0), 'carry': (1, 1), 'temporary': (2, 2)}[kind]
        seconds = (length / c['linear_speed'] + self._turns(routes, base.yaw) /
                   c['angular_speed'] + grasps * c['grasp_seconds'] + places * c['place_seconds'])
        work = (length * c['rolling_force'] + mass * c['gravity'] *
                self.geometry['lift_height'] * grasps +
                mass * carried_distance * c['carry_force_per_kg'])
        score = (c['time_weight'] * seconds / c['time_reference'] +
                 c['work_weight'] * work / c['work_reference'])
        return seconds, work, score

    def _reachable(self, base, object_pose, position_tolerance=0.0):
        # The virtual arm has an explicitly configured spherical work envelope
        # centred at the configured mount height; this is not the URDF arm IK.
        arm_z = self.geometry.get('arm_mount_height', 0.6)
        return math.sqrt((distance(base, object_pose) + position_tolerance)**2 +
                         (object_pose.z - base.z - arm_z)**2) <= self.geometry['arm_reach']

    def _sweep_clear(self, begin, end, obj, environment):
        """Conservative XY volume check; never assumes an object clears a wall in Z."""
        steps = max(1, math.ceil(distance(begin, end) / self.geometry['manipulation_step']))
        raw = self.grid.overlay(environment, exclude_ids=(obj.object_id,))
        for i in range(steps + 1):
            t = i / steps
            p = Pose3(begin.x + (end.x - begin.x) * t,
                      begin.y + (end.y - begin.y) * t,
                      begin.z + (end.z - begin.z) * t,
                      begin.yaw + wrap(end.yaw - begin.yaw) * t)
            if not self.grid._placement_raw(
                    p, obj.size, raw, self.geometry['grasp_clearance']):
                return False
        return True

    def choose(self, base, goal, obstacle, known_obstacles=(), excluded=frozenset(), now_ns=None):
        base, goal = _pose(base), _pose(goal)
        if obstacle.frame != self.grid.frame:
            return None, [{'kind': 'all', 'feasible': False, 'reason': 'frame_mismatch'}]
        environment = {o.object_id: o for o in known_obstacles}
        environment[obstacle.object_id] = obstacle
        environment = tuple(environment.values())
        logs, candidates = [], []

        def excluded_kind(kind):
            return (kind in excluded or (obstacle.object_id, kind) in excluded or
                    f'{obstacle.object_id}:{kind}' in excluded)

        def record(kind, reason, **detail):
            logs.append({'kind': kind, 'object_id': obstacle.object_id,
                         'feasible': False, 'reason': reason, **detail})

        if not excluded_kind('detour'):
            route = self.grid.route(base, goal, environment)
            if route:
                seconds, work, score = self._cost('detour', [route], base)
                candidate = Candidate('detour', obstacle.object_id, route=tuple(route),
                                      time_cost=seconds, work_cost=work, score=score,
                                      routes={'detour': tuple(route)})
                candidates.append(candidate)
                logs.append({'kind': 'detour', 'feasible': True, 'object_id': obstacle.object_id,
                             'time': seconds, 'work': work, 'score': score})
            else:
                record('detour', 'no_route')
        else:
            record('detour', 'excluded')

        policy = self.classes.get(obstacle.label, {})
        reason = None
        if not policy.get('movable', False):
            reason = 'class_not_movable'
        elif obstacle.confidence < self.perception['min_confidence']:
            reason = 'low_confidence'
        elif now_ns is not None and (not isinstance(now_ns, int) or now_ns < obstacle.stamp_ns or
                now_ns - obstacle.stamp_ns > self.perception['max_observation_age_sec'] * 1e9):
            reason = 'observation_stale_or_future'
        mass = policy.get('mass')
        if reason is None and (not isinstance(mass, (int, float)) or
                               not math.isfinite(mass) or mass <= 0):
            reason = 'mass_not_configured'
        if reason is not None:
            record('manipulation', reason)
            return self._select(candidates), logs

        kind = 'carry' if policy.get('carry', False) else 'temporary'
        if excluded_kind(kind):
            record(kind, 'excluded')
            return self._select(candidates), logs
        g, original = self.geometry, obstacle.pose
        def reachable(base_pose, target_pose):
            return self._reachable(base_pose, target_pose, g['base_position_tolerance'])

        # Progress direction is from the stopped robot through the object, not
        # from an arbitrary map/world axis or a YOLO label's assumed orientation.
        direction = math.atan2(original.y - base.y, original.x - base.x)
        if distance(original, base) <= 1e-6:
            record(kind, 'base_inside_target')
            return self._select(candidates), logs
        extent = math.hypot(*obstacle.size[:2]) / 2
        required_pass = extent + self.grid.config['robot_radius'] + self.grid.config['safety_margin']
        if g['pass_distance'] <= required_pass or g['action_distance'] <= required_pass:
            record(kind, 'action_or_restore_overlaps_target_envelope')
            return self._select(candidates), logs
        backend = self.config.get('backend', {})
        carry_x, carry_y = backend.get('carry_x', 0.0), backend.get('carry_y', 0.0)
        extra = max(0.0, math.hypot(carry_x, carry_y) + extent + g['grasp_clearance'] -
                    self.grid.config['robot_radius'])
        remaining = tuple(o for o in environment if o.object_id != obstacle.object_id)
        if not self.grid.placement_free(original, obstacle.size, remaining,
                                        clearance=g['grasp_clearance']):
            record(kind, 'original_pose_not_restorable_in_static_map')
            return self._select(candidates), logs

        for index, angle_offset in enumerate(g['action_angles']):
            approach_angle = direction + angle_offset
            ax, ay = math.cos(approach_angle), math.sin(approach_angle)
            dx, dy = math.cos(direction), math.sin(direction)
            action = Pose3(original.x - g['action_distance'] * ax,
                           original.y - g['action_distance'] * ay, 0.0, approach_angle)
            restore = Pose3(original.x + g['pass_distance'] * dx,
                            original.y + g['pass_distance'] * dy, 0.0, wrap(direction + math.pi))
            action, restore = self.grid.snapped_pose(action), self.grid.snapped_pose(restore)
            detail = {'candidate_index': index, 'action': (action.x, action.y, action.yaw),
                      'restore': (restore.x, restore.y, restore.yaw)}
            lifted = replace(original, z=original.z + g['lift_height'])
            if not all((reachable(action, original), reachable(action, lifted),
                        reachable(restore, original), reachable(restore, lifted))):
                record(kind, 'virtual_arm_reach', **detail)
                continue
            approach = self.grid.route(base, action, environment)
            departure = self.grid.route(restore, goal, environment)
            if not approach or not departure:
                record(kind, 'approach_or_after_restore_unreachable', **detail)
                continue
            carry_action = Pose3(action.x + carry_x * math.cos(action.yaw) -
                                 carry_y * math.sin(action.yaw),
                                 action.y + carry_x * math.sin(action.yaw) +
                                 carry_y * math.cos(action.yaw),
                                 backend.get('carry_z', g['carry_height']), original.yaw)
            carry_restore = Pose3(restore.x + carry_x * math.cos(restore.yaw) -
                                  carry_y * math.sin(restore.yaw),
                                  restore.y + carry_x * math.sin(restore.yaw) +
                                  carry_y * math.cos(restore.yaw),
                                  carry_action.z, original.yaw)
            if not reachable(action, carry_action) or not reachable(restore, carry_restore):
                record(kind, 'carry_pose_out_of_reach', **detail)
                continue
            if not (self._sweep_clear(original, carry_action, obstacle, remaining) and
                    self._sweep_clear(carry_restore, original, obstacle, remaining)):
                record(kind, 'manipulation_sweep_blocked', **detail)
                continue
            if kind == 'carry':
                crossing = self.grid.route(action, restore, remaining, extra_radius=extra)
                rollback = self.grid.route(restore, action, remaining, extra_radius=extra)
                if not crossing or not rollback:
                    record(kind, 'carry_or_rollback_unreachable', **detail)
                    continue
                routes = {'approach': tuple(approach), 'crossing': tuple(crossing),
                          'departure': tuple(departure)}
                self._add_candidate(candidates, logs, kind, obstacle, action, restore, None,
                                    routes, rollback, base, mass, route_length(crossing), extra, detail)
                continue
            for offset in g['temporary_offsets']:
                temporary = Pose3(original.x - dy * offset, original.y + dx * offset,
                                  original.z, original.yaw)
                temp_detail = {**detail, 'temporary': (temporary.x, temporary.y)}
                if not reachable(action, temporary) or not reachable(restore, temporary):
                    record(kind, 'temporary_not_reachable_from_both_sides', **temp_detail)
                    continue
                if not self.grid.placement_free(temporary, obstacle.size, remaining,
                                                clearance=g['grasp_clearance']):
                    record(kind, 'temporary_occupied_or_unknown', **temp_detail)
                    continue
                if not (self._sweep_clear(original, temporary, obstacle, remaining) and
                        self._sweep_clear(temporary, original, obstacle, remaining)):
                    record(kind, 'temporary_manipulation_sweep_blocked', **temp_detail)
                    continue
                temp_environment = (*remaining, replace(obstacle, pose=temporary))
                crossing = self.grid.route(action, restore, temp_environment)
                rollback = self.grid.route(restore, action, temp_environment)
                if not crossing or not rollback:
                    record(kind, 'temporary_crossing_or_rollback_unreachable', **temp_detail)
                    continue
                routes = {'approach': tuple(approach), 'crossing': tuple(crossing),
                          'departure': tuple(departure)}
                self._add_candidate(candidates, logs, kind, obstacle, action, restore, temporary,
                                    routes, rollback, base, mass, 0.0, 0.0, temp_detail)
        return self._select(candidates), logs

    def _add_candidate(self, candidates, logs, kind, obj, action, restore, temporary,
                       routes, rollback, base, mass, carried, extra, detail):
        seconds, work, score = self._cost(kind, list(routes.values()), base, mass, carried)
        route = tuple(p for leg in routes.values() for p in leg)
        candidate = Candidate(kind, obj.object_id, action, restore, restore, temporary,
                              obj.pose, route, seconds, work, score, routes,
                              tuple(rollback), extra)
        candidates.append(candidate)
        logs.append({'kind': kind, 'object_id': obj.object_id, 'feasible': True,
                     'time': seconds, 'work': work, 'score': score, **detail})

    @staticmethod
    def _select(candidates):
        # Python's stable min keeps deterministic enumeration order after ties.
        return min(candidates, key=lambda c: (c.score, 0 if c.kind == 'detour' else 1),
                   default=None)


def observation_fresh(obstacle, now_ns, maximum_age_sec):
    """Shared strict freshness check; future observations fail closed."""
    return (isinstance(now_ns, int) and math.isfinite(maximum_age_sec) and
            maximum_age_sec > 0 and 0 <= now_ns - obstacle.stamp_ns <= maximum_age_sec * 1e9)


def pose_error(actual, expected):
    """Position error in metres and shortest yaw error in radians."""
    return (math.sqrt((actual.x - expected.x)**2 + (actual.y - expected.y)**2 +
                      (actual.z - expected.z)**2), abs(wrap(actual.yaw - expected.yaw)))
