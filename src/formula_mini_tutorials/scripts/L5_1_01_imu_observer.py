#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
L5-1 IMU观察器：观察加速度计、陀螺仪数据
验证IMU内部状态与陀螺仪漂移现象

或是直接在终端使用命令查看数据：
rostopic echo /tianracer/imu


"""

import rospy
from sensor_msgs.msg import Imu
import tf.transformations
import math

class IMUObserver:
    
    def __init__(self):
        # 初始化节点
        rospy.init_node('l5_1_imu_observer', anonymous=True)
        # 订阅IMU数据
        rospy.Subscriber('/tianracer/imu', Imu, self.imu_callback)
        
        # # 控制打印频率
        # self.frame_count = 0
        # self.print_interval = 50  # 每50帧打印一次

        self.last_time = rospy.Time.now()
        
        rospy.loginfo("[IMU观察器]")
    
    def imu_callback(self, msg):
        """处理IMU消息"""

        # # 控制打印频率
        # self.frame_count += 1
        # if self.frame_count % self.print_interval != 0:
        #     return
        
        # 提取加速度
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
        
        # 提取角速度
        omega_x = msg.angular_velocity.x
        omega_y = msg.angular_velocity.y
        omega_z = msg.angular_velocity.z
        

        
        rospy.loginfo("[IMU] 加速度:(%.3f,%.3f,%.3f)m/s² |角速度:(%.3f,%.3f,%.3f)rad/s°" % (ax, ay, az, omega_x, omega_y, omega_z))
        
        # self.last_time = current_time


def main():
    observer = IMUObserver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
