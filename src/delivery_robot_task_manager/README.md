# delivery_robot_task_manager

The task manager subscribes to `/task_manager/task_requests`, validates the
batch, plans pickup/delivery order and cargo slots, then exclusively orchestrates
the chassis, arm, and cargo Actions.

`config/mission_sites.yaml` is intentionally disabled until home, locker staging,
and all locker slot face poses have been measured. A slot pose yaw represents
the locker outward normal. Delivery staging poses are supplied directly by each
`Order` and do not use receiver precision alignment in the current baseline.

After side alignment, open a locker slot manually with:

```bash
ros2 service call /locker/set_door_state \
  delivery_robot_interfaces/srv/SetLockerDoor \
  "{locker_id: locker_1, locker_slot: 1, open: true}"
```

While `StoreTray` is active, the manager may send a `PREPARE_ROUTE` goal to the
chassis. That operation only computes a Nav2 path. Chassis motion is strictly
sequential and cannot start until arm and cargo both report
`transport_safe=true`.
