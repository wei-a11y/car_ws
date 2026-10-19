调整lqr的步骤
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