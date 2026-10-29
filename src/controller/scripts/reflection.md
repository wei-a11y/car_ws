调整lqr的步骤初版
1.先开启记录路径的文件，准备记录路径
colcon build
source install/setup.bash
python3 src/controller/scripts/record_path_tracking.py
2.开启仿真
新开终端
ros2 launch car_description navigation.launch.py \
  controller_type:=lqr lqr_path_topic:=/plan
3.给出固定目标位置，便于对lqr的调整
再开一个新终端
ros2 topic pub --once --use-sim-time --qos-durability volatile \
  /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: map},
    pose: {
      position: {x: -0.87, y: 7.64, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }}"
4.通过图像分析小车运行曲线，然后来看如何修改lqr
在新终端再执行
python3 src/controller/scripts/analyze_path_tracking.py

调整lqr第二版
1.ros2 launch controller path_tracking.launch.py \
  params_file:="$PWD/src/controller/config/path_tracking.yaml"
2.ros2 launch car_description navigation.launch.py controller_type:=lqr \
  motion_limits_file:="$PWD/src/plan/config/motion_limits.yaml" \
  planner_params_file:="$PWD/src/plan/config/astar.yaml" \
  lqr_params_file:="$PWD/src/controller/config/lqr_tracker.yaml"
3.ros2 topic pub --once --use-sim-time --qos-durability volatile \
  /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: map},
    pose: {
      position: {x: -4.142, y: 7.919, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }}"
4.python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml

5.调整lqr第三版
1.ros2 launch controller path_tracking.launch.py \
  params_file:="$PWD/src/controller/config/path_tracking.yaml"
2.ros2 launch car_description navigation.launch.py controller_type:=lqr \
  motion_limits_file:="$PWD/src/plan/config/motion_limits.yaml" \
  planner_params_file:="$PWD/src/plan/config/astar.yaml" \
  lqr_params_file:="$PWD/src/controller/config/lqr_tracker.yaml"
3.ros2 topic pub --once --use-sim-time --qos-durability volatile \
  /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: map},
    pose: {
      position: {x: -4.142, y: 7.919, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }}"
4.python3 src/controller/scripts/analyze_path_tracking.py \
  --config src/controller/config/path_tracking.yaml \
  --data-dir data/path_record/run_具体时间/path_0001 --no-show

问题
1.需要时刻注意小车的位置，可用ros2 topic echo /amcl_pose --once查看小车实时位置
2.需要注意当小车在墙体膨胀半径周围时，很容易无法定位运动，这需要初始化小车的位置，将其远离膨胀区域
3.修改坐标文件时，需要区分清除哪个是map的，哪个是rviz的，哪个是gazebo的，哪个是小车的，可以借助gpt定位在哪个文件修改
4.在map的坐标更改之后，需要先手动将小车初始化位置，否则小车的robot模型会出现错误
5.在使用这个命令时候，不要随意更改orientation里面的数字，里面是四元数，会要求每个平方之和等于1（根据A*代码文件规定的，可以修改），不然小车无法运动。命令如下
ros2 topic pub --once --use-sim-time --qos-durability volatile \
  /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: map},
    pose: {
      position: {x: -0.87, y: 7.64, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }}"
6.在我运行python3 src/controller/scripts/analyze_path_tracking.py指令后报错，一般都是因为旧文件残留，导致分析文件读取的路径是旧文件的路径，此时要么直接指令读取到最新产生的图像进行分析；要么就修改分析文件里读取文件的路径

如何根据图像曲线判断小车情况然后进行修改
| 图中现象 | 优先怀疑的环节 | 应重点检查 |
|---|---|---|
| 参考速度就反复升降，指令和实际速度跟着变化 | 参考限速、预瞄与制动处理 | 是否在相邻弯之间反复解除、重新施加限速；曲率是否频繁起伏 |
| 参考速度较平滑，但指令速度反复变化 | 纵向控制及指令约束 | PID 输出、反馈噪声、限幅和加速度约束之间的影响 |
| 指令速度平滑，实际速度明显滞后或振荡 | 底盘响应与反馈 | 加减速能力、底盘速度控制、里程计反馈 |
| 曲率在连接处突变或出现尖峰 | 路径几何处理 | 圆弧与过渡段是否连续，采样和参考点选择是否稳定 |
| 前馈角速度平滑，指令角速度却明显左右摆动 | 横向反馈或参考方向 | LQR 修正量、航向参考、投影点是否跳变 |
| 转弯时线速度接近零，仍有明显角速度 | 停车转向或重新对准 | 控制状态、必要停车标记，以及此前的制动过程 |