#!/usr/bin/env python
# coding:utf-8 
import rospy
import copy
import os
import csv
import rospkg
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class PathPlanning():
    def wpts_cb(self, msg):
        # 1. 保存到文件（原有逻辑保留）
        try:
            with open(self.file_path, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([msg.point.x, msg.point.y, msg.point.z])
            rospy.loginfo("Waypoint saved: [%.2f, %.2f]", msg.point.x, msg.point.y)
        except IOError as e:
            rospy.logerr("Failed to write to file: %s", e)

        # 2. 可视化标记（核心修复）
        pt = Marker()  # 不再用self.pt，避免类变量覆盖
        pt.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.marker_frame
        pt.header.stamp = rospy.Time.now()
        pt.ns = "waypoints"  # 命名空间更清晰
        pt.id = self.i       # 唯一ID，避免覆盖
        pt.action = Marker.ADD
        pt.type = Marker.CYLINDER

        # 标记位置（和点击点一致）
        pt.pose.position.x = msg.point.x
        pt.pose.position.y = msg.point.y
        pt.pose.position.z = msg.point.z  # 如果地图是2D，可设为0.1（避免贴地看不见）
        
        pt.pose.orientation.w = 1.0
        # 放大尺寸，确保肉眼可见（原0.05m太小）
        pt.scale.x = 0.2    # 底面直径0.2m
        pt.scale.y = 0.2
        pt.scale.z = 0.4    # 高度0.4m
		
        # 红色不透明
        pt.color.r = 1.0
        pt.color.g = 0.0
        pt.color.b = 0.0
        pt.color.a = 1.0    # 必须设置透明度，否则RViz默认透明（看不见）
        
        # 关键：设置标记永久显示（直到节点关闭）
        pt.lifetime = rospy.Duration()  # 空Duration表示永久
        
        # 修复：每次只发布当前新标记（而非整个数组），更高效且不易出错
        single_marker_array = MarkerArray()
        single_marker_array.markers.append(pt)
        self.marker_pub.publish(single_marker_array)

        self.i += 1  # ID自增

    def __init__(self):
        rospy.init_node('path_planning', anonymous=False)
        
        # 确定文件保存路径
        package_path = rospkg.RosPack().get_path('formula_mini_tutorials')
        self.file_path = os.path.join(package_path, 'data', 'waypoints.csv')
        self.robot_name = rospy.get_param('~robot_name', 'tianracer')
        self.marker_frame = rospy.get_param('~marker_frame', f'{self.robot_name}/map')
        self.clicked_point_topic = rospy.get_param('~clicked_point_topic', '/clicked_point')
        self.clicked_point_topic_ns = rospy.get_param('~clicked_point_topic_ns', f'/{self.robot_name}/clicked_point')
        
        # 初始化时清空文件
        with open(self.file_path, 'w') as f:
            f.truncate()
        rospy.loginfo("Waypoints file initialized at: %s", self.file_path)

        # 修复：增大队列大小，避免消息丢失
        self.marker_pub = rospy.Publisher("way_points", MarkerArray, queue_size=10)
        self.way_point_sub = rospy.Subscriber(self.clicked_point_topic, PointStamped, self.wpts_cb, queue_size=10)
        self.way_point_sub_ns = None
        if self.clicked_point_topic_ns != self.clicked_point_topic:
            self.way_point_sub_ns = rospy.Subscriber(self.clicked_point_topic_ns, PointStamped, self.wpts_cb, queue_size=10)
        
        # 成员变量
        self.i = 0  # ID从0开始（ROS标记ID建议从0起始）

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        planner = PathPlanning()
        rospy.loginfo("Path planning node started, click in RViz to add waypoints!")
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted, exiting...")
        pass
