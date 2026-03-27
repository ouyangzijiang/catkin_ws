#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

class ReactiveAvoidLRNode(object):
    def __init__(self):
        rospy.init_node('l3_1_reactive_avoid_lr')
        
        # --- 参数设置 ---
        self.steer_K = float(rospy.get_param('~steer_K', 1.0)) # 转向比例系数
        self.forward_speed = float(rospy.get_param('~speed_mps', 0.8)) # 前进速度 (米/秒)

        # --- 状态变量 ---
        self.target_speed = 0.0
        self.target_steer = 0.0
        self.last_scan_time = rospy.Time.now()

        # --- 通信 ---
        self.pub_cmd = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=10)
        rospy.Subscriber('/tianracer/scan', LaserScan, self.on_scan_callback, queue_size=1)
        # 30Hz 高频发布
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)

        rospy.loginfo(">>> 左右避障节点启动: Speed=%.1f, K=%.1f", self.forward_speed, self.steer_K)


    # 计算应该怎么走
    def on_scan_callback(self, scan):
        self.last_scan_time = rospy.Time.now()
        n = len(scan.ranges)

        # --- 1. 定义扇区 ---
        cnt_5  = int((5.0  / 360.0) * n)
        cnt_35 = int((35.0 / 360.0) * n)

        # 左右扇区用于“转向”
        lf_ranges = scan.ranges[cnt_5 : cnt_35]
        rf_ranges = scan.ranges[-cnt_35 : -cnt_5]

        # 正前方扇区用于“停车” (看中间 +/- 15度)
        cnt_front_width = int((15.0 / 360.0) * n)
        front_part_a = scan.ranges[0 : cnt_front_width]
        front_part_b = scan.ranges[-cnt_front_width : ]
        front_ranges = front_part_a + front_part_b

        # --- 2. 数据清洗 ---
        def get_valid_min(ranges_list, debug_name):
            # 过滤掉 0.05 以下(盲区) 和 >10.0(太远) 和 inf
            valid = [r for r in ranges_list if 0.05 < r < 10.0 and math.isfinite(r)]
            if not valid: return 2.0 
            return min(valid)

        left_dist  = get_valid_min(lf_ranges, "Left")
        right_dist = get_valid_min(rf_ranges, "Right")
        front_dist = get_valid_min(front_ranges, "Front")

        # --- 3. 速度决策逻辑 ---
        real_min_obstacle = min(left_dist, right_dist, front_dist)
        STOP_DIST = 0.3 

        if real_min_obstacle < STOP_DIST:
            # 停车模式
            self.target_speed = 0.0
            # 【修改点】停车时，把方向回正（0.0），防止舵机乱响乱动
            self.target_steer = 0.0
            rospy.logwarn_throttle(0.5, "危险! 距离(%.2fm) < %.2f -> 停车并回正", real_min_obstacle, STOP_DIST)
        else:
            # 前进模式
            self.target_speed = self.forward_speed
            
            # --- 4. 只有在前进时才更新转向逻辑 ---
            # 转向逻辑：根据左右距离差异调整转向，离左近就往右转，离右近就往左转
            #容易过度反应，所以加个比例系数 K 来调节转向强度,容易震荡
            diff = left_dist - right_dist
            steer_cmd = self.steer_K * diff
            
            # 限幅
            max_steer = 0.5
            steer_cmd = max(min(steer_cmd, max_steer), -max_steer)
            self.target_steer = steer_cmd


    # 持续发命令
    def timer_callback(self, event):
        # 超时保护
        if (rospy.Time.now() - self.last_scan_time).to_sec() > 0.5:
            self.target_speed = 0.0
            self.target_steer = 0.0 # 超时停车也要回正
            rospy.logwarn_throttle(2.0, "雷达断连 -> 停车")

        msg = AckermannDrive()
        msg.speed = self.target_speed
        msg.steering_angle = self.target_steer
        
        self.pub_cmd.publish(msg)


def main():
    node = ReactiveAvoidLRNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()