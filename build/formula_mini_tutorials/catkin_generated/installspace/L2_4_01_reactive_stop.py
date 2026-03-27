#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

# 反应式停止节点
class ReactiveStopNode:
    def __init__(self):
        # 初始化 ROS 节点和参数
        rospy.init_node('l2_4_reactive_stop')
        # 距离阈值
        self.threshold = float(rospy.get_param('~threshold_m', 0.5))
        # 前进速度
        self.forward_speed = float(rospy.get_param('~forward_speed_mps', 0.7)) 

        # 发布器和订阅器
        self.pub_cmd = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=10)
        # 加上 queue_size=1 保证数据最新
        rospy.Subscriber('/tianracer/scan', LaserScan, self.on_scan_callback, queue_size=1)
        
        # --- 状态变量 ---
        self.target_speed = 0.0      # 当前期望速度
        self.last_scan_time = rospy.Time.now() # 上次收到雷达数据的时间
        
        # 创建一个 30Hz (每秒30次) 的定时器，用于持续发送控制指令
        # 这样即使雷达频率低（比如10Hz），也能“填补”中间的空窗期，防止底层超时刹车
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        
        rospy.loginfo(">>> 自动刹车节点启动 (Timer定时器模式): 阈值=%.2f米", self.threshold)


    # 雷达回调：只负责“做决定”（更新目标速度），不发命令
    def on_scan_callback(self, scan):
        # 1. 更新收到数据的时间戳 (就是为了证明雷达还活着)
        self.last_scan_time = rospy.Time.now()

        # 2. 提取前方数据 (首尾拼接)
        n = len(scan.ranges)
        range_width = int((45.0 / 360.0) * n)
        part_a = scan.ranges[0 : range_width]
        part_b = scan.ranges[-range_width : ]
        front_ranges = part_a + part_b 
        
        # 3. 过滤
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0 and r != float('inf')]

        # 4. 决策逻辑
        if not valid_ranges:
            # 贴脸或无数据 -> 期望停车
            self.target_speed = 0.0
        else:
            min_dist = min(valid_ranges)
            if min_dist < self.threshold:
                # 遇障 -> 期望停车
                self.target_speed = 0.0
            else:
                # 安全 -> 期望前进
                self.target_speed = self.forward_speed


    # 定时器回调：高频发布指令 + “安全检查”
    def timer_callback(self, event):
        # 1. 超时保护：如果超过 0.5秒 没收到雷达数据 (雷达断连/卡死)
        #    即使定时器还在跑，也不能继续瞎跑了，必须停车
        dt = (rospy.Time.now() - self.last_scan_time).to_sec()
        
        if dt > 0.5:
            self.target_speed = 0.0
            rospy.logwarn_throttle(2.0, "警告：雷达数据中断 (%.2fs 无数据)，强制停车！", dt)

        # 2. 持续发布控制指令 (填补空窗期)
        msg = AckermannDrive()
        msg.speed = self.target_speed
        msg.steering_angle = 0.0
        self.pub_cmd.publish(msg)

        # 状态打印 (throttle 防止刷屏)
        if self.target_speed == 0.0:
            rospy.logwarn_throttle(1.0, "状态: 停车 (Speed=0)")
        else:
            rospy.loginfo_throttle(2.0, "状态: 前进 (Speed=%.2f)", self.target_speed)


def main():
    node = ReactiveStopNode()
    # spin 会维持住节点，让 Timer 和 Subscriber 持续工作
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()