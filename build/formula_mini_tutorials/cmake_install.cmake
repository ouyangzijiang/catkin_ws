# Install script for directory: /home/tianbot/catkin_ws/src/formula_mini_tutorials

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tianbot/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/formula_mini_tutorials.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/formula_mini_tutorials/cmake" TYPE FILE FILES
    "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/formula_mini_tutorialsConfig.cmake"
    "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/formula_mini_tutorialsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/formula_mini_tutorials" TYPE FILE FILES "/home/tianbot/catkin_ws/src/formula_mini_tutorials/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_1_01_pub_chatter.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_1_02_sub_chatter_spin.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_1_03_sub_chatter_no_spin.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_2_01_ackermann_action.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_2_03_ackermann_left_stop.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_2_04_ackermann_right_stop.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_3_01_scan_front_min.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_3_02_scan_front_threshold.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_3_03_scan_5sectors_min.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_4_01_reactive_stop.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_4_02_reactive_follow.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L2_4_03_reactive_avoid_lr.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L3_1_01_scan_5sectors_filter.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L3_2_01_wall_follow_right.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L3_3_01_follow_the_gap.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L3_4_01_disparity_extender.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L5_1_01_imu_observer.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L5_1_02_odom_observer.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L6_2_01_astar_demo.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L6_4_01_pure_pursuit.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L6_4_02_l1_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L6_4_03_waypoint_recorder.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/formula_mini_tutorials" TYPE PROGRAM FILES "/home/tianbot/catkin_ws/build/formula_mini_tutorials/catkin_generated/installspace/L6_4_04_waypoint_dispatcher.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/formula_mini_tutorials/launch" TYPE DIRECTORY FILES "/home/tianbot/catkin_ws/src/formula_mini_tutorials/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/formula_mini_tutorials/config" TYPE DIRECTORY FILES "/home/tianbot/catkin_ws/src/formula_mini_tutorials/config/")
endif()

