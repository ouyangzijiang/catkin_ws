#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan

# 前方距离阈值（米）
threshold = 0.5 

class L2_3_front_threshold:
    def __init__(self):
        rospy.init_node('L2_3_front_threshold')
        # 添加 queue_size=1 防止延迟
        rospy.Subscriber("/tianracer/scan", LaserScan, self.on_scan_callback, queue_size=1)
        rospy.loginfo(f">>> L2-3: 阈值警报监听中 (报警距离: {threshold} 米)...")

    # 当话题有新消息时会自动触发回调函数
    def on_scan_callback(self, msg):
        n = len(msg.ranges)
        
        # 这里的逻辑是左右各 45 度，总共 90 度
        # 45 / 360 * n
        range_width = int((45.0 / 360.0) * n)
        
        part_a = msg.ranges[0 : range_width]       # 0 ~ 45度
        part_b = msg.ranges[-range_width : ]       # 315 ~ 360度
        front_ranges = part_a + part_b             # 拼接

        # 2.简单分析：找最近的障碍物，滤波掉无效数据
        valid_ranges = [r for r in front_ranges if 0.05 < r < 10.0 and r != float('inf')]
        
        if valid_ranges:
            min_dist = min(valid_ranges)
            if min_dist < threshold:
                rospy.logwarn(f"前方障碍物过近！距离: {min_dist:.2f} 米 < 阈值 {threshold} 米")
            else:
                rospy.loginfo(f"前方障碍物距离: {min_dist:.2f} 米")
        else:
            # 列表为空通常是因为离得太近进入盲区了，或者是极为空旷
            rospy.logwarn("无有效数据 (可能已贴入盲区 或 极度空旷)")

def main():
    scanner = L2_3_front_threshold()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()