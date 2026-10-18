# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
# Auxiliary tracking semantics. /plan remains nav_msgs/Path.
# The complete associated Path permits exact pairing even when DDS topics arrive out of order.
# Arrays have exactly path.poses.size() elements. Angles are radians; curvature is signed 1/metre.
nav_msgs/Path path
float64[] tangent_yaw
float64[] curvature
bool[] stop_required
