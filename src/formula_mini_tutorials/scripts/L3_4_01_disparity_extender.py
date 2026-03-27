#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
"""L3-4 跟随空隙算法（Follow the Gap）较比于 L3-2 的墙跟随算法，最大的不同在于
它有膨胀功能：看到障碍物时，不仅把它自己视为墙，还把它周围一定角度范围内的区域也视为墙。LaserScan
   ↓
前方 ±60°
   ↓
数据清洗
   ↓
障碍物膨胀（新增）
   ↓
二值化（路/墙）
   ↓
找最大Gap
   ↓
取中心
   ↓
转向控制"""
class L3_3_Follow_Gap:
    def __init__(self):
        rospy.init_node('l3_3_follow_gap')
        
        # --- 参数 ---
        self.speed = 0.6          # 巡航速度
        self.limit = 1.2          # 障碍物阈值：大于此距离才认为是路
        self.safety_dist = 0.25   # 紧急贴脸停车距离
        self.inflation_dist = 0.2 # 膨胀半径 (车宽/2 + 安全余量)

        # --- 状态 ---
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.last_scan_time = rospy.Time.now()

        # 发布与订阅
        self.pub = rospy.Publisher("/tianracer/ackermann_cmd", AckermannDrive, queue_size=1)
        rospy.Subscriber("/tianracer/scan", LaserScan, self.on_scan_callback, queue_size=1)
        # 引入定时器 (30Hz) 解决电机抖动/不走问题
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)

    def on_scan_callback(self, msg):
        self.last_scan_time = rospy.Time.now()
        n = len(msg.ranges)
        
        # 1. 切片：获取正前方 +/- 60 度的视野
        # 注意：TianRacer 雷达 0度是正前方(Index 0)，所以需要拼接数组首尾
        width = n // 6  # 60度对应的点数
        right_part = msg.ranges[n - width : n] # -60度 ~ 0度
        left_part  = msg.ranges[0 : width]     # 0度 ~ +60度
        view_ranges = right_part + left_part   # 拼接成 [-60, +60] 连续数组

        # 2. 安全检查：贴脸停车
        # 过滤无效数据，取出正前方一小块区域检查
        clean_view = [r if (0.05 < r < 10.0 and math.isfinite(r)) else 10.0 for r in view_ranges]
        min_front = min(clean_view[int(len(clean_view)*0.3) : int(len(clean_view)*0.7)])
        
        if min_front < self.safety_dist:
            self.cmd_speed = 0.0
            self.cmd_steer = 0.0
            return

        # 车有宽度，看到障碍点时，应该把它周围一定角度内的区域都视为不可通行
        inflated_view = list(clean_view) # 复制一份用于修改
        angle_inc = msg.angle_increment
        
        for i, r in enumerate(clean_view):
            # 如果该点本身是障碍 (距离小于阈值)
            if r <= self.limit:
                # 计算需要把周围多少个点也“染”黑
                #膨胀半径 = 车宽/2 + 安全边距
                # 公式: arctan(膨胀半径 / 距离) / 角分辨率
                # 距离越近，遮挡角度越大
                dist = max(r, 0.01) # 避免除以0
                angle_width = math.atan(self.inflation_dist / dist) 
                idx_width = int(angle_width / angle_inc)
                
                # 将波及范围内的点设为 0 (墙)
                start = max(0, i - idx_width)
                end = min(len(inflated_view), i + idx_width + 1)
                for k in range(start, end):
                    inflated_view[k] = 0.0

        # 3. 找“路” (Binary Map)
        # 大于 limit 认为是 1 (Gap/路)，否则是 0 (Wall/墙)
        # 注意：这里改用 inflated_view
        binary_map = [1 if r > self.limit else 0 for r in inflated_view]

        # 4. 找最长连续的 1 (Max Gap)
        max_len, best_start = 0, 0
        curr_len, curr_start = 0, 0
        
        for i, val in enumerate(binary_map):
            if val == 1:
                if curr_len == 0: curr_start = i
                curr_len += 1
            else:
                if curr_len > max_len:
                    max_len = curr_len
                    best_start = curr_start
                curr_len = 0
        # 检查收尾
        if curr_len > max_len:
            max_len = curr_len
            best_start = curr_start

        # 5. 计算导航目标
        target_steer = 0.0
        target_speed = self.speed

        if max_len > 10: # 路够宽才走
            # 算出 Gap 中心索引
            center_idx = best_start + max_len / 2.0
            
            # 索引映射回角度：
            # view_ranges 长度为 2*width。中点(width)对应 0度。
            # 归一化偏移量 (-1.0 ~ 1.0)
            mid_point = len(view_ranges) / 2.0
            offset = (center_idx - mid_point) / mid_point 
            
            target_steer = offset * 1.5 # 乘系数增加转向灵敏度
        else:
            target_speed = 0.0 # 无路可走
            rospy.logwarn_throttle(1.0, "No Gap Found!")

        # 6. 更新指令 (最低速度保护)
        if target_speed > 0.01 and target_speed < 0.5:
            target_speed = 0.5

        self.cmd_speed = target_speed
        self.cmd_steer = max(-1.0, min(1.0, target_steer))

    def timer_callback(self, event):
        if (rospy.Time.now() - self.last_scan_time).to_sec() > 0.5:
            self.cmd_speed = 0.0
            self.cmd_steer = 0.0
            
        msg = AckermannDrive()
        msg.speed = self.cmd_speed
        msg.steering_angle = self.cmd_steer
        self.pub.publish(msg)

def main():
    node = L3_3_Follow_Gap()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()