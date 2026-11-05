一、开启lqr实车运行
1.ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
2.ros2 launch rplidar_ros rplidar_a1_launch.py   serial_port:=/dev/ttyUSB1 serial_baudrate:=115200 frame_id:=laser_link
3.ros2 run src odom2tf
4.ros2 launch car_description navigation_real.launch.py   map_yaml:=/home/dx/tzcup/car_ws/map1/room.yaml   use_safety_gate:=false enable:=true cmd_vel_topic:=/cmd_vel

二、记录图像
1.python3 src/controller/scripts/record_path_tracking.py --ros-args \
  --params-file src/controller/config/path_tracking.yaml \
  -p use_sim_time:=false
  2.python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml \
  --data-dir data/path_record/run_本次时间/path_0001 \
  --no-show
