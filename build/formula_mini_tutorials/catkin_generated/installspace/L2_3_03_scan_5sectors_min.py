#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 Python 示例：五扇区最小距离（前/左前/右前/左/右）
- 体现内容：将 /scan 分为 5 个方位扇区，各自过滤无效值并计算最小距离；打印一行五个数，也可分别发布到话题。
- 教学用途：建立“多方位距离”直觉，为 Day3 扇区化处理铺垫；本节仍以观察为主，不做控制。

参数（私有）：
- ~half_width_deg（float，默认 15.0）：每个扇区的半宽（度）。
- 扇区中心（度）：Front=0，LeftFront=+30，RightFront=-30，Left=+90，Right=-90。
- ~min_valid_m（float，默认 0.05）
- ~max_valid_m（float，默认 20.0）
- ~publish_float（bool，默认 True）：是否发布到各自 Float32 话题。

"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

#   定义五个扇区及其中心角度，每60度一个扇区，覆盖前180度（从左到右：L / LF / F / RF / R）
SECTORS = [
    ("left", 90.0),
    ("left_front", 30.0),
    ("front", 0.0),
    ("right_front",330.0),
    ("right", 270.0),
]

# 计算指定扇区内的最小有效距离  
def sector_min(scan, center_deg, half_deg, min_valid, max_valid):
    '''在指定扇区内计算最小有效距离。'''
    #   角度转换为弧度
    center_rad = math.radians(center_deg)
    half_rad = math.radians(half_deg)
    #   计算扇区起止角度
    start_rad = center_rad - half_rad
    end_rad = center_rad + half_rad

    #   遍历激光扫描数据，筛选扇区内的有效距离
    angle = scan.angle_min
    inc = scan.angle_increment
    ranges = scan.ranges

    #   存储最小有效距离
    best = float('inf')
    #   遍历所有测距点
    for i, r in enumerate(ranges):
        #   当前角度
        a = angle + i * inc
        #   判断是否在扇区内
        if start_rad <= a <= end_rad:
            #   过滤无效与越界
            if math.isfinite(r) and r >= min_valid and r <= max_valid:
                #   同时参考激光自身的 range_min / range_max
                if r >= scan.range_min and r <= scan.range_max:
                    #   更新最小值
                    if r < best:
                        best = r
    return best

# 五扇区最小距离节点
class FiveSectorsNode(object):
    # 初始化
    def __init__(self):
        # ROS 节点初始化与参数读取
        rospy.init_node('l2_3_5sectors_min')
        self.half_deg = float(rospy.get_param('~half_width_deg', 15.0)) # 每个扇区的半宽（度）
        self.min_valid = float(rospy.get_param('~min_valid_m', 0.05))  # 过滤点距小于 5 厘米的读数，通常是雷达盲区内的无效数据
        self.max_valid = float(rospy.get_param('~max_valid_m', 20.0))#过滤 点大于20米的读数，通常是极度空旷或无穷远的无效数据
        # 订阅激光扫描数据---接收到消息后触发回调
        rospy.Subscriber('/tianracer/scan', LaserScan, self.on_scan_callback)

    # 扫描回调
    def on_scan_callback(self, scan):

        # 计算各扇区最小距离
        results = {}
        #   遍历各扇区
        for name, center in SECTORS:
            # 计算当前扇区最小距离
            dmin = sector_min(scan, center, self.half_deg, self.min_valid, self.max_valid)
            #  存储结果
            results[name] = dmin

        # 打印一行结果，便于观察不同方位变化（L / LF / F / RF / R）
        fmt = 'L={:.2f} LF={:.2f} F={:.2f} RF={:.2f} R={:.2f}'
        #   打印结果
        rospy.loginfo(fmt.format(results['left'], results['left_front'], results['front'], results['right_front'], results['right']))


def main():
    node = FiveSectorsNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
