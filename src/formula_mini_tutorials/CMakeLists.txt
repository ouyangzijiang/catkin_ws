cmake_minimum_required(VERSION 3.0.2)
project(formula_mini_tutorials)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  ackermann_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/L2_1_01_pub_chatter.py
  scripts/L2_1_02_sub_chatter_spin.py
  scripts/L2_1_03_sub_chatter_no_spin.py
  scripts/L2_2_01_ackermann_action.py
  scripts/L2_2_03_ackermann_left_stop.py
  scripts/L2_2_04_ackermann_right_stop.py
  scripts/L2_3_01_scan_front_min.py
  scripts/L2_3_02_scan_front_threshold.py
  scripts/L2_3_03_scan_5sectors_min.py
  scripts/L2_4_01_reactive_stop.py
  scripts/L2_4_02_reactive_follow.py
  scripts/L2_4_03_reactive_avoid_lr.py
  scripts/L3_1_01_scan_5sectors_filter.py
  scripts/L3_2_01_wall_follow_right.py
  scripts/L3_3_01_follow_the_gap.py
  scripts/L3_4_01_disparity_extender.py
  scripts/L5_1_01_imu_observer.py
  scripts/L5_1_02_odom_observer.py
  scripts/L6_2_01_astar_demo.py
  scripts/L6_4_01_pure_pursuit.py
  scripts/L6_4_02_l1_controller.py
  scripts/L6_4_03_waypoint_recorder.py
  scripts/L6_4_04_waypoint_dispatcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
