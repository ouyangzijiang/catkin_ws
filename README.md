# formula_mini_tutorials

Formula Mini 课程教学例程 ROS 包。供学生使用，详细参数需要根据实际车辆调节。
安装Cartographer：https://blog.csdn.net/weixin_44362628/article/details/122540297 ,后进行：
安装regulated_pure_pursuit_controller：https://github.com/mitukou1109/regulated_pure_pursuit_controller

## 安装

```bash
cd ~/catkin_ws/src
git clone <repo_url> formula_mini_tutorials
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

运行
在车机启动
```bash
export ROS_IP=车机IP
export ROS_MASTER_URI=http://车机IP:11311
export ROS_IP=192.168.128.143
export ROS_MASTER_URI=http://192.168.128.10:11311
roslaunch tianracer_bringup tianracer_bringup.launch
```
在本机运行
```bash
export ROS_IP=本机IP
export ROS_MASTER_URI=http://车机IP:11311
source ~/catkin_ws/devel/setup.bash
```

## 目录结构

```
formula_mini_tutorials/
├── CMakeLists.txt
├── package.xml
├── scripts/          # 所有 Python 脚本
├── launch/           # Launch 文件
├── config/           # RViz / 参数配置文件
├── src/              # C++ 源码
└── docs/             # 补充文档
```

## 脚本索引

### L2 — ROS 基础与控车

| 脚本 | 说明 |
|------|------|
| `L2_1_01_pub_chatter.py` | 发布节点 |
|运行|rosrun formula_mini_tutorials L2_1_01_pub_chatter.py|
| `L2_1_02_sub_chatter_spin.py` | 订阅节点（使用 spin） |
|运行|rosrun formula_mini_tutorials L2_1_02_sub_chatter_spin.py|
| `L2_1_03_sub_chatter_no_spin.py` | 订阅节点（不使用 spin） |
|运行|rosrun formula_mini_tutorials L2_1_03_sub_chatter_no_spin.py|
| `L2_2_01_ackermann_action.py` | 阿克曼控车模板（可调速度/转角/时长） |
|运行|rosrun formula_mini_tutorials L2_2_01_ackermann_action.py _speed_mps:=0.3 _steering_angle_deg:=0 _duration_sec:=1.5|
| `L2_2_03_ackermann_left_stop.py` | 车辆左转 |
|运行|rosrun formula_mini_tutorials L2_2_03_ackermann_left_stop.py _speed_mps = 0.8 _steering_angle_deg = 40.0 _duration_sec = 1.0|
| `L2_2_04_ackermann_right_stop.py` | 车辆右转 |
|运行|rosrun formula_mini_tutorials L2_2_04_ackermann_right_stop.py _speed_mps = 0.8 _steering_angle_deg = -40.0 _duration_sec = 3.0|
| `L2_3_01_scan_front_min.py` | 前方扇区最小距离 |
|运行|rosrun formula_mini_tutorials L2_3_01_scan_front_min.py|
| `L2_3_02_scan_front_threshold.py` | 前方扇区阈值判断 |
|运行|rosrun formula_mini_tutorials L2_3_02_scan_front_threshold.py |
| `L2_3_03_scan_5sectors_min.py` | 五扇区距离显示 |
|运行|rosrun formula_mini_tutorials L2_3_03_scan_5sectors_min.py|
| `L2_4_01_reactive_stop.py` | 前方阈值刹停 |
|运行|rosrun formula_mini_tutorials L2_4_01_reactive_stop.py _threshold_m:=0.5 _forward_speed_mps:=0.7|
| `L2_4_02_reactive_follow.py` | 跟随目标 |
|运行|rosrun formula_mini_tutorials L2_4_02_reactive_follow.py|
| `L2_4_03_reactive_avoid_lr.py` | 左右避障转向 |
|运行|rosrun formula_mini_tutorials L2_4_02_reactive_follow.py|

### L3 — 反应式导航

| 脚本 | 说明 |
|------|------|
| `L3_1_01_scan_5sectors_filter.py` | 雷达五扇区滤波数据清洗 |
|运行|rosrun formula_mini_tutorials L3_1_01_scan_5sectors_filter.py|
| `L3_2_01_wall_follow_right.py` | 巡墙程序 |
|运行|rosrun formula_mini_tutorials L3_2_01_wall_follow_right.py |
| `L3_3_01_follow_the_gap.py` | Follow-the-Gap 简单逻辑实现 |
|运行|rosrun formula_mini_tutorials L3_3_01_follow_the_gap.py |
| `L3_4_01_disparity_extender.py` | Follow-the-Gap 膨胀处理 |
|运行|rosrun formula_mini_tutorials L3_4_01_disparity_extender.py|
### L5 — 传感器与建图

| 脚本 | 说明 |
|------|------|
| `L5_1_01_imu_observer.py` | IMU 数据获取 |
|运行| rosrun formula_mini_tutorials L5_1_01_imu_observer.py|
| `L5_1_02_odom_observer.py` | 里程计数据获取 |
|运行|rosrun formula_mini_tutorials L5_1_02_odom_observer.py|

### L6 — 路径规划与跟踪

| 脚本 | 说明 |
|------|------|
| `L6_2_01_astar_demo.py` | A\*/Dijkstra 路径规划可视化演示（无需 ROS） |
|运行|rosrun formula_mini_tutorials L6_2_01_astar_demo.py |
| `L6_4_01_pure_pursuit.py` | Pure Pursuit 路径跟踪 |
|运行|rosrun formula_mini_tutorials L6_4_01_pure_pursuit.py |
| `L6_4_02_l1_controller.py` | L1 路径跟踪控制器（Python 教学版） |
|运行|rosrun formula_mini_tutorials L6_4_02_l1_controller.py |
| `L6_4_03_waypoint_recorder.py` | RViz 点击记录路径点到 CSV |
|运行|rosrun formula_mini_tutorials L6_4_03_waypoint_recorder.py|
| `L6_4_04_waypoint_dispatcher.py` | 从 CSV 依次发送目标点 |
|运行|rosrun formula_mini_tutorials L6_4_04_waypoint_dispatcher.py |
## Launch 文件

| 文件 | 说明 |
|------|------|
| `L2_1_pubsub_demo.launch` | Pub/Sub 示例一键启动 |
|运行|roslaunch formula_mini_tutorials L2_1_pubsub_demo.launch|
| `L5_2_cartographer.launch` | Cartographer 建图 |
|运行|roslaunch formula_mini_tutorials L5_2_cartographer.launch,后打开rviz找map话题|
| `L6_1_map_load.launch` | 加载地图文件（教学用） |
|运行|roslaunch formula_mini_tutorials L6_1_map_load.launch map_file:=/home/tianbot/mymap.yaml|
| `L6_4_l1_nav.launch` | L1 路径跟踪导航（地图+定位+规划+L1控制） |
|运行|roslaunch formula_mini_tutorials L6_4_l1_nav.launch|
| `L6_4_pure_pursuit_nav.launch` | Pure Pursuit 导航 |
|运行||
| `L6_4_view_rviz.launch` | RViz 可视化 |
|运行|roslaunch formula_mini_tutorials L6_4_view_rviz.launch|

## 运行示例

```bash
# Pub/Sub 示例
roslaunch formula_mini_tutorials L2_1_pubsub_demo.launch

# 单独运行脚本
rosrun formula_mini_tutorials L2_2_01_ackermann_action.py _speed_mps:=0.3 _steering_angle_deg:=0 _duration_sec:=1.5
```