# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
# Auxiliary tracking semantics. /plan remains nav_msgs/Path.
# The complete associated Path permits exact pairing even when DDS topics arrive out of order.
# Arrays have exactly path.poses.size() elements. Angles are radians; curvature is signed 1/metre.
nav_msgs/Path path
float64[] tangent_yaw
float64[] curvature
bool[] stop_required
# True: nodal curvature interpolates continuously between samples, except across stops.
# False: legacy outgoing-primitive curvature is held on each segment.
bool curvature_continuous
# Optional extended metadata: all four arrays empty for a legacy producer, or all length N.
# Values at i certify outgoing segment i -> i+1; terminal speed_limit is zero.
uint8 LINE=0
uint8 TRANSITION=1
uint8 ARC_FALLBACK=2
uint8[] segment_type
float64[] speed_limit
float64[] curvature_bound
float64[] curvature_rate_bound
# JSON planning summary and per-corner rejection counts; diagnostic only.
string processing_report
