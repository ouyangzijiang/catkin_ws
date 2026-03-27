#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 Python 示例：右转一小段 → 自动停止
- 体现内容：向 /ackermann_cmd 发布低速右转命令（转向角为负），持续 T 秒后自动停止
- 安全提示：保持低速与短时长；如转向过急或停不稳，立即调小参数

默认参数（可通过 rosparam 修改）：
- ~speed_mps = 0.8
- ~steering_angle_deg = -40.0  # 右转为负角度（度），内部转为弧度
- ~duration_sec = 3.0
"""

import math
import rospy
from ackermann_msgs.msg import AckermannDrive


def main():
    rospy.init_node('l2_2_right_stop')
# 获取参数
    speed_mps = float(rospy.get_param('~speed_mps', 0.8))
    steering_deg = float(rospy.get_param('~steering_angle_deg', -40.0))
    duration_sec = float(rospy.get_param('~duration_sec', 3))
#发布转向命令
    pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=10)


    steer_rad = math.radians(steering_deg)
    end_time = rospy.Time.now() + rospy.Duration(duration_sec)
    rate = rospy.Rate(50)

    rospy.loginfo('Right-turn start: speed=%.3f m/s, steering=%.3f deg (%.3f rad), duration=%.2f s',
                  speed_mps, steering_deg, steer_rad, duration_sec)
# 持续发布转向命令，直到达到指定时间
    while rospy.Time.now() < end_time:
        msg = AckermannDrive()
        msg.speed = speed_mps
        msg.steering_angle = steer_rad
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo('Publishing STOP...')
    rospy.loginfo('Done.')


if __name__ == '__main__':
    main()
