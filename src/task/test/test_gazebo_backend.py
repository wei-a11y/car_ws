# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Simulated service tests: fresh observations, transformations and gripper state."""
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Time as TimeMsg
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import TransformStamped
import pytest

from task_runtime.gazebo_backend import (
    GazeboBackend, load_registry, ros_pose, transform_pose)
from task_runtime.recovery_core import Pose3
from task_runtime.scenario import model_sdf


REGISTRY = Path(__file__).resolve().parents[1]/'config'/'recovery_scenarios.yaml'


class Future:
    def __init__(self, value=None, done=True):
        self.value, self.complete, self.cancelled = value, done, False

    def done(self):
        return self.complete

    def result(self):
        return self.value

    def cancel(self):
        self.cancelled = True


class World:
    def __init__(self):
        self.time = 10.0
        self.tf = TransformStamped()
        self.tf.header.frame_id, self.tf.child_frame_id = 'map', 'base_link'
        self.tf.transform.translation.x, self.tf.transform.translation.y = 4., -3.
        self.tf.transform.translation.z = 0.1
        self.tf.transform.rotation.z = math.sin(math.pi/8)
        self.tf.transform.rotation.w = math.cos(math.pi/8)
        self.objects = {'task_recovery_bag': Pose3(4.5, -2.5, 0.22, 0.4)}
        self.requests, self.sets = [], []
        self.markers = []
        self.service_ready, self.move_objects, self.drop_get = True, True, False
        self.exact_transform_stamps = []

    @property
    def nanoseconds(self):
        return int(self.time*1e9)

    def to_msg(self):
        return TimeMsg(sec=int(self.time), nanosec=int((self.time % 1)*1e9))

    def now(self):
        return self

    def get_clock(self):
        return self

    def lookup_transform(self, target, source, stamp):
        assert target == 'map' and source == 'base_link'
        self.exact_transform_stamps.append(stamp.nanoseconds)
        self.tf.header.stamp = self.to_msg()
        return self.tf

    def create_client(self, service, name):
        return SimpleNamespace(service_is_ready=lambda: self.service_ready,
                               call_async=self.get if service is GetEntityState else self.set)

    def create_publisher(self, *_):
        return SimpleNamespace(publish=self.markers.append)

    def get(self, request):
        self.requests.append(request)
        if self.drop_get:
            return Future(done=False)
        result = GetEntityState.Response()
        result.header.stamp = self.to_msg()
        result.header.frame_id = request.reference_frame
        if request.name == 'car::base_link':
            result.success = True
            result.state.pose.orientation.w = 1.0
        elif request.name in self.objects:
            result.success = True
            result.state.pose = ros_pose(transform_pose(self.objects[request.name], self.tf.transform, True))
        return Future(result)

    def set(self, request):
        self.sets.append(request)
        assert request.state.name.startswith('task_recovery_')
        assert request.state.reference_frame == 'car::base_link'
        if self.move_objects:
            p, q = request.state.pose.position, request.state.pose.orientation
            self.objects[request.state.name] = transform_pose(
                Pose3(p.x, p.y, p.z, 2*math.atan2(q.z, q.w)), self.tf.transform)
        return Future(SetEntityState.Response(success=True))


@pytest.fixture
def system(monkeypatch):
    world = World()
    monkeypatch.setattr('task_runtime.gazebo_backend.time.monotonic', lambda: world.time)
    config = dict(backend_registry_file=str(REGISTRY), backend_poll_sec=0.02,
                  backend_timeout_sec=0.18, action_step_sec=0.02,
                  observation_timeout_sec=0.15, lift_speed=5., lift_height=0.25,
                  carry_x=0., carry_y=0., carry_z=0.8, verification_samples=2)
    backend = GazeboBackend(world, config, world)
    advance(world, backend, 3)
    assert backend.ready()[0]
    return world, backend


def advance(world, backend, count=1):
    for _ in range(count):
        world.time += 0.025
        backend.tick()


def finish(world, backend, count=250):
    for _ in range(count):
        advance(world, backend)
        if backend.result is not None:
            return backend.result
    pytest.fail(f'Operation did not finish: {backend.operation}')


def begin(world, backend):
    visible = backend.observations(lambda _: True)
    assert len(visible) == 1
    assert backend.begin_grasp(visible[0])
    return visible[0]


def test_nonidentity_reference_transform_and_hidden_truth(system):
    world, backend = system
    assert backend.observations(lambda _: False) == []
    assert backend.get_observation('bag') is None
    visible = backend.observations(lambda _: True)
    assert visible[0].pose == pytest.approx(world.objects['task_recovery_bag'])
    assert visible[0].frame == 'map'
    assert any(stamp > 0 for stamp in world.exact_transform_stamps)
    assert len(backend.registry) > len(visible), 'Absent fixtures must not block readiness'


def test_full_lift_carry_and_restore_observations(system):
    world, backend = system
    original = begin(world, backend).pose
    stages = []
    for _ in range(250):
        stages.append(backend.operation)
        advance(world, backend)
        if backend.result:
            break
    assert backend.result['code'] == 'GRASP_VERIFIED'
    assert backend.held_id == 'bag'
    assert {'OPEN', 'APPROACH', 'CLOSE', 'LIFT', 'VERIFY_GRASP', 'MOVE_TO_CARRY', 'VERIFY_CARRY'} <= set(stages)
    # Lift commands precede transport commands and stay above the same map XY.
    first = world.sets[0].state.pose
    p = transform_pose(Pose3(first.position.x, first.position.y, first.position.z,
                            2*math.atan2(first.orientation.z, first.orientation.w)), world.tf.transform)
    assert p.x == pytest.approx(original.x)
    assert p.y == pytest.approx(original.y)
    assert p.z > original.z
    assert backend.begin_place(original, restore=True)
    assert finish(world, backend)['code'] == 'RESTORE_VERIFIED'
    assert backend.held_id is None and backend.gripper_open
    restored = world.objects['task_recovery_bag']
    assert restored.x == pytest.approx(original.x)
    assert restored.y == pytest.approx(original.y)
    assert restored.z == pytest.approx(original.z)
    assert restored.yaw == pytest.approx(original.yaw)


def test_service_success_without_target_motion_is_failure(system):
    world, backend = system
    world.move_objects = False
    begin(world, backend)
    result = finish(world, backend)
    assert not result['success'] and result['code'] == 'GRASP_NOT_FOLLOWING'


def test_injected_grasp_failure_retracts_then_retry_succeeds(system):
    world, backend = system
    backend.registry['bag']['grasp_failures'] = 1
    begin(world, backend)
    assert finish(world, backend)['code'] == 'GRASP_NOT_FOLLOWING'
    assert backend.held_id is None and backend.gripper_open
    assert not world.sets, 'A failed virtual closure must not move the object'
    begin(world, backend)
    assert finish(world, backend)['success']
    assert backend.attempts['bag'] == 2


def test_cancel_preserves_attachment_and_follows_base(system):
    world, backend = system
    begin(world, backend)
    assert finish(world, backend)['success']
    backend.cancel()
    assert backend.held_id == 'bag' and backend.result['code'] == 'CANCELLED'
    before = world.objects['task_recovery_bag']
    world.tf.transform.translation.x += 0.3
    advance(world, backend, 3)
    after = world.objects['task_recovery_bag']
    assert after.x-before.x == pytest.approx(0.3)
    assert after.y == pytest.approx(before.y)


def test_stale_target_and_lost_services_fail_closed(system):
    world, backend = system
    obstacle = backend.observations(lambda _: True)[0]
    world.time += 1.
    assert not backend.begin_grasp(obstacle)
    advance(world, backend, 4)
    begin(world, backend)
    world.service_ready = False
    advance(world, backend)
    assert backend.result['code'] == 'BACKEND_LOST'
    assert not backend.ready()[0]


def test_restore_requires_new_observations_after_release(system):
    world, backend = system
    original = begin(world, backend).pose
    assert finish(world, backend)['success']
    assert backend.begin_place(original, restore=True)
    for _ in range(250):
        advance(world, backend)
        if backend.operation == 'VERIFY_PLACE':
            break
    assert backend.operation == 'VERIFY_PLACE' and backend.gripper_open
    world.drop_get = True
    backend.pending.clear()
    backend.cache.clear()
    result = finish(world, backend)
    assert not result['success']


def test_registry_and_sdf_only_allow_test_models(tmp_path):
    registry = load_registry(REGISTRY)
    root = ET.fromstring(model_sdf(registry['bag']))
    assert root.find('model').attrib['name'] == 'task_recovery_bag'
    assert root.find('.//collision') is not None
    assert root.find('.//gravity').text == 'false'
    path = tmp_path/'bad.yaml'
    path.write_text('objects:\n - {object_id: car, model_name: car, label: bag, size: [1, 1, 1]}\n')
    with pytest.raises(ValueError, match='task_recovery_'):
        load_registry(path)


def test_out_of_reach_and_unauthorized_grasps(system):
    world, backend = system
    obstacle = backend.cache['bag'][0]
    assert not backend.begin_grasp(obstacle)
    assert backend.result['code'] == 'UNAUTHORIZED_OBJECT'
    world.objects['task_recovery_bag'] = Pose3(9., 9., 0.22, 0.)
    advance(world, backend, 4)
    obstacle = backend.observations(lambda _: True)[0]
    assert not backend.begin_grasp(obstacle)
    assert backend.result['code'] == 'UNREACHABLE_GRASP'


def test_restore_pose_error_does_not_count_as_success(system):
    world, backend = system
    original = begin(world, backend).pose
    assert finish(world, backend)['success']
    assert backend.begin_place(original, restore=True)
    for _ in range(250):
        advance(world, backend)
        if backend.operation == 'VERIFY_PLACE':
            break
    world.objects['task_recovery_bag'] = Pose3(original.x + 0.2, original.y, original.z)
    result = finish(world, backend)
    assert not result['success'] and result['code'] == 'RESTORE_NOT_VERIFIED'


def test_missing_timestamped_tf_does_not_expose_world_truth(system):
    world, backend = system

    def unavailable(*args):
        raise RuntimeError('no transform at measurement time')

    world.lookup_transform = unavailable
    advance(world, backend, 20)
    assert not backend.ready()[0]
    assert backend.observations(lambda _: True) == []
