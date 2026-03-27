#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
L5-1 里程计观察器：观察odom话题中的位置、速度数据
验证里程计漂移现象

或是直接在终端使用命令查看数据：
rostopic echo /tianracer/odom
"""

import rospy
from nav_msgs.msg import Odometry
import tf.transformations
import math

class OdomObserver:
    
    def __init__(self):
        # 初始化节点
        rospy.init_node('l5_1_odom_observer', anonymous=True)
        # 订阅里程计数据
        rospy.Subscriber('/tianracer/odom', Odometry, self.odom_callback)
        
        # self.frame_count = 0
        # self.print_interval = 20  # 每20帧打印一次
        
        rospy.loginfo("[Odom观察器]")
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        # # 控制打印频率
        # self.frame_count += 1
        # if self.frame_count % self.print_interval != 0:
        #     return
        
        # 提取位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # 提取姿态（四元数→欧拉角）
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = math.degrees(euler[2])
        
        # 提取速度
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega_z = msg.twist.twist.angular.z
        
        linear_speed = math.sqrt(vx**2 + vy**2)
        # print(msg)
        rospy.loginfo("[Odom] 位置: (%.3f, %.3f) | 姿态: %.2f° | 速度: %.3f m/s, %.3f rad/s" % (x, y, yaw, linear_speed, omega_z))


def main():
    observer = OdomObserver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
