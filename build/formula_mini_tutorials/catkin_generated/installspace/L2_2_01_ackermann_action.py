#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 Python 示例：最小控车（单次动作）
- 体现内容：向 /ackermann_cmd 发布 AckermannDrive 命令，持续 T 秒后自动发布停止（零速）
- 用途场景：课堂演示“前进/左转/右转后停”，强调安全优先与参数化控制
- 关键点：
  - 采用 ackermann_msgs/AckermannDrive，话题为 /ackermann_cmd
  - 回调不涉及；本节点仅为发布者（Publisher）
  - 参数化三要素：速度（m/s）、转向角（度）、持续时间（秒）
  - 开始与结束均发布停止命令，确保“停得住”

运行前提：
- 已启动 roscore，并在车辆端切换到 ROS 控制模式（遥控随时可接管）
- 建议先在“轮子悬空”状态验证停止命令有效，再上地执行

可调私有参数（rosparam 或命令行设置）：
- ~speed_mps（float，默认 0.3）：线速度，单位 m/s
- ~steering_angle_deg（float，默认 0.0）：转向角，单位 度，将在程序内转为弧度
- ~duration_sec（float，默认 1.5）：动作持续时间，单位 秒
"""

import math
import rospy
from ackermann_msgs.msg import AckermannDrive



# 主运行函数
def run_action():
    # 节点初始化
    rospy.init_node('l2_2_ackermann_action')

    # 读取参数：速度、转角（度）、持续时间
    speed_mps = float(rospy.get_param('~speed_mps', 0.7))
    steering_deg = float(rospy.get_param('~steering_angle_deg', 0.0))
    duration_sec = float(rospy.get_param('~duration_sec', 2))

    # 发布者（/ackermann_cmd）
    pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=10)#这里需要使用rostopic list 查看具体话题名称 注意命名空间



    # 准备动作参数
    steering_rad = math.radians(steering_deg)
    end_time = rospy.Time.now() + rospy.Duration(duration_sec)
    rate = rospy.Rate(50)  # 20Hz 发布命令

    rospy.loginfo('Action start: speed=%.3f m/s, steering=%.3f deg (%.3f rad), duration=%.2f s',speed_mps, steering_deg, steering_rad, duration_sec)

    # 在持续时间内循环发布控制命令
    while rospy.Time.now() < end_time:
        msg = AckermannDrive()
        msg.speed = speed_mps
        msg.steering_angle = steering_rad
        pub.publish(msg)
        rate.sleep()

    # 动作结束后发布停止
    rospy.loginfo('Action end. Publishing STOP...')
    rospy.loginfo('Done.')


if __name__ == '__main__':
    run_action()
