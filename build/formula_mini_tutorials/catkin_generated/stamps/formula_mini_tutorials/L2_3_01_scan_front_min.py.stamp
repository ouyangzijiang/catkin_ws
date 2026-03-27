#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan

class L2_3_scan_front_min:
    def __init__(self):
        rospy.init_node('L2_3_scan_front_min')
        # 订阅雷达数据
        rospy.Subscriber("/tianracer/scan", LaserScan, self.on_scan_callback,queue_size=1)
        rospy.loginfo(">>> L2-3: 雷达监听中...")

    # 当话题有新消息时会自动触发回调函数
    def on_scan_callback(self, msg):
        # 1. 数据切片：只看前方 45 度 (-22.5 ~ +22.5)
        n = len(msg.ranges)
        # 计算 22.5 度对应的索引宽度
        # 22.5 / 360 * n
        range_width = int((22.5 / 360.0) * n)

        # 正前方是 0 度（数组首尾）
        # 前方数据 = 数组开头的一段(0 ~ range_width) 与 数组结尾的一段(-range_width ~ end)做拼接
        part_a = msg.ranges[0 : range_width]
        part_b = msg.ranges[-range_width : ]
        front_ranges = part_a + part_b  
        # （新列表 = 原列表[起始位置 : 结束位置]）
        
        # 2.简单分析：找最近的障碍物
        # 过滤掉 0.0 和 inf
        # 保留范围	0.1 ~ 10.0 米之间的有效距离数据
        # 2. 分类处理
        # 先把 0.0 和 inf 去掉，只看真实的有效物理读数
        # 注意：雷达盲区内的数据通常会变成 0.0，或者是极小值
        real_readings = [r for r in front_ranges if r > 0.05 and r != float('inf')]
         
        if not real_readings:
            # 如果列表为空，说明前方全是盲区(0.0)或者全是无穷远(inf)
            # 这种情况下通常意味着贴得很近（进入盲区）或者彻底空旷
            rospy.logwarn("!!! 警告：无法检测有效距离 (可能过近入盲区) !!!")
            return
        # 找最小距离
        min_dist = min(real_readings)

        # 3. 根据距离分级打印
        if min_dist < 0.2:  # 如果小于 20 厘米
            rospy.logwarn(f"!!! 警告：距离过近 !!! ({min_dist:.2f} 米)")
        elif min_dist < 10.0:
            rospy.loginfo(f"前方障碍物距离: {min_dist:.2f} 米")
        else:
            rospy.loginfo("前方空旷 (超出量程)")

def main():
    scanner = L2_3_scan_front_min()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()