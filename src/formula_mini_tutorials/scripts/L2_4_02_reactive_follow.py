#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

class ReactiveFollowNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('l2_4_reactive_follow')

        # --- 参数设置 ---
        self.d_target = float(rospy.get_param('~d_target', 0.5))  # 目标保持 0.5 米
        self.K = float(rospy.get_param('~K', 1.0))                # 比例系数 P (反应灵敏度)
        self.v_max = float(rospy.get_param('~v_max', 1.0))        # 最大限制速度

        # --- 状态变量 ---
        self.target_speed = 0.0
        self.last_scan_time = rospy.Time.now()

        # --- 通信接口 ---
        self.pub_cmd = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=10)
        
        # queue_size=1 降低延迟
        rospy.Subscriber('/tianracer/scan', LaserScan, self.on_scan_callback, queue_size=1)

        # 使用 30Hz 定时器高频发布命令，防止小车顿挫
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)

        rospy.loginfo(">>> 自动跟车启动 (目标距离: %.2fm, Kp: %.2f)...", self.d_target, self.K)
    def on_scan_callback(self, scan):
    
        self.last_scan_time = rospy.Time.now()

        # 方向切片 (0度为正前方，切首尾)
        n = len(scan.ranges)
        range_width = int((22.5 / 360.0) * n) # 左右各22.5度，视角比较窄，方便跟车
        
        part_a = scan.ranges[0 : range_width]
        part_b = scan.ranges[-range_width : ]
        front_ranges = part_a + part_b 
        
        # 降低过滤门槛，防止贴脸变盲
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0 and r != float('inf')]

        # --- 控制算法 ---
        if not valid_ranges:
            # 如果前面什么都没有（通过调整角度，跟车时一般不会全空）
            # 或者贴脸贴死了（盲区）
            # 安全策略：立刻停车
            self.target_speed = 0.0
            rospy.logdebug("无有效数据 -> 停车等待")
        else:
            # 找到最近的点作为跟车目标
            current_dist = min(valid_ranges)

            # 计算误差： 正数=太远(要追), 负数=太近(要退/停)
            error = current_dist - self.d_target
            
            # P 控制计算速度
            # 举例：目标0.5m，实测1.0m，误差0.5m，K=1.0 -> 速度 0.5 m/s
            v_cmd = self.K * error

            # 速度限幅 (Clamp)
            # 限制在 -v_max 到 +v_max 之间
            v_cmd = max(min(v_cmd, self.v_max), -self.v_max)

            # 简单的死区设置 (防止在目标位置震荡)
            if abs(error) < 0.05: # 误差小于 5厘米 就不动了
                v_cmd = 0.0

            self.target_speed = v_cmd

            # 调试打印 (限制频率)
            rospy.loginfo_throttle(1.0, "Dist: %.2fm | Err: %.2f | CmdSpeed: %.2f", current_dist, error, v_cmd)
    def timer_callback(self, event):
        # 安全检查：如果雷达断连超过 0.5秒，强制停车
        dt = (rospy.Time.now() - self.last_scan_time).to_sec()
        if dt > 0.5:
            self.target_speed = 0.0
            rospy.logwarn_throttle(2.0, "雷达超时 (%.2fs) -> 强制停车", dt)

        # 发布指令
        msg = AckermannDrive()
        msg.speed = self.target_speed
        msg.steering_angle = 0.0  # 暂时只跟直线
        
        self.pub_cmd.publish(msg)

def main():
    node = ReactiveFollowNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()