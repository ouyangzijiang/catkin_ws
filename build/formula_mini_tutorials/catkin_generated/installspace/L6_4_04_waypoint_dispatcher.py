#!/usr/bin/env python3
# coding:utf-8 
import rospy
import os
import csv
import math
import tf
from geometry_msgs.msg import PoseStamped

class WaypointDispatcher():
    def __init__(self):
        rospy.init_node('waypoint_dispatcher', anonymous=False)
        
        self.robot_name = rospy.get_param('~robot_name', 'tianracer')
        self.file_path = '/home/sunrise/waypoints.csv'
        
        # Publisher
        topic_name = f"/{self.robot_name}/move_base_simple/goal"
        self.goal_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
        self.listener = tf.TransformListener()
        
        self.wpts = []
        self.current_index = 0
        self.goal_sent = False
        self.goal_send_time = None  # 记录发送goal的时间
        # 匹配L1的goal_radius，减小判定阈值
        self.reach_threshold = 0.05  # 改为0.05米，和L1的goal_radius一致

    def load_waypoints(self):
        if not os.path.exists(self.file_path):
            rospy.logerr("Waypoints file not found at: %s", self.file_path)
            return False
        
        try:
            with open(self.file_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 2:
                        self.wpts.append([float(row[0]), float(row[1])])
            
            rospy.loginfo("Loaded %d waypoints from file.", len(self.wpts))
            return True
        except Exception as e:
            rospy.logerr("Failed to read waypoints: %s", e)
            return False

    def run(self):
        if not self.load_waypoints():
            return

        rate = rospy.Rate(10)
        rospy.loginfo("Start dispatching goals...")
        
        while self.goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo_throttle(2.0, "Waiting for subscriber to connect to %s", self.goal_pub.name)
            rospy.sleep(0.5)

        while not rospy.is_shutdown():
            if self.current_index < len(self.wpts):
                target_x = self.wpts[self.current_index][0]
                target_y = self.wpts[self.current_index][1]

                # 1. 发送目标
                if not self.goal_sent:
                    goal = PoseStamped()
                    goal.header.frame_id = f"{self.robot_name}/map"
                    goal.header.stamp = rospy.Time.now()
                    goal.pose.position.x = target_x
                    goal.pose.position.y = target_y
                    goal.pose.orientation.w = 1.0
                    
                    self.goal_pub.publish(goal)
                    self.goal_sent = True
                    self.goal_send_time = rospy.Time.now()  # 记录发送时间
                    rospy.loginfo("Sending goal %d/%d: (%.2f, %.2f)", 
                                  self.current_index + 1, len(self.wpts), target_x, target_y)

                # 2. 检查距离（增加延迟+匹配L1阈值）
                try:
                    # 必须等待1秒后再检查，给L1足够时间处理goal
                    if self.goal_send_time and (rospy.Time.now() - self.goal_send_time).to_sec() > 1.0:
                        (trans, rot) = self.listener.lookupTransform(f'{self.robot_name}/map', 
                                                                     f'{self.robot_name}/base_link', 
                                                                     rospy.Time(0))
                        dx = trans[0] - target_x
                        dy = trans[1] - target_y
                        dist = math.sqrt(dx**2 + dy**2)
                        rospy.logdebug("Current distance to goal %d: %.3f m", self.current_index+1, dist)

                        # 判定阈值改为0.05米，和L1的goal_radius一致
                        if dist < self.reach_threshold:
                            rospy.loginfo("Reached goal %d (distance: %.3f m)", self.current_index + 1, dist)
                            self.current_index += 1
                            self.goal_sent = False
                            rospy.sleep(0.5) # 停顿0.5秒，避免频繁切换

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn("TF lookup failed: %s", e)
            else:
                rospy.loginfo_once("All goals reached!")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        dispatcher = WaypointDispatcher()
        dispatcher.run()
    except rospy.ROSInterruptException:
        pass
