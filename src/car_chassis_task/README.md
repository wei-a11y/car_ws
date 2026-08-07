# car_chassis_task

This package owns high-level chassis motion through `/chassis/execute_task`.
Pickup execution is `Nav2 approach -> target search -> side-facing precision
alignment -> hold`. Delivery poses are staging-only in the current baseline.

The station target pose uses `map`; its +x axis must be the outward normal from
the station face into the aisle. The selected robot side and all offsets are in
`config/side_alignment.yaml`.

Manual locker door input:

```bash
ros2 service call /locker/set_door_state \
  delivery_robot_interfaces/srv/SetLockerDoor \
  "{locker_id: locker_1, locker_slot: 1, open: true}"
```

Nav2 publishes `/cmd_vel`. The velocity arbiter forwards either Nav2 or
precision-alignment commands to `/cmd_vel_safe`, which is the only input to the
diff-drive controller.
