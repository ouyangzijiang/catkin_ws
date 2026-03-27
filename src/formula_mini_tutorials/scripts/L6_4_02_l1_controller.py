#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
L1 路径跟踪控制器（Python 教学版）

功能：订阅全局路径和里程计，用 L1 导航算法计算转向角，发布 AckermannDrive 指令。
原理：
  1. 从 move_base 规划的全局路径中，找到车前方距离 >= Lfw 的第一个"前视点"
  2. 计算车头到前视点的方位角 eta
  3. 用 L1 公式把 eta 转换为转向角
  4. 以固定速度 + 计算出的转向角驱动车辆

话题：
  订阅：
    - odometry/filtered        (nav_msgs/Odometry)   里程计
    - move_base/NavfnROS/plan  (nav_msgs/Path)       全局路径（launch 中 remap）
    - move_base_simple/goal    (geometry_msgs/PoseStamped) RViz 目标点
  发布：
    - ackermann_cmd            (ackermann_msgs/AckermannDrive) 车辆控制
    - car_path                 (visualization_msgs/Marker)     RViz 可视化

参数（可通过 rosparam / launch 覆盖）：
    ~robot_name       (str,   default: 从环境变量 TIANRACER_NAME 或 "tianracer")
    ~L                (float, default: 0.26)   轴距 (m)
    ~lfw              (float, default: 0.13)   前轮到质心偏移 (m)
    ~Vcmd             (float, default: 1.0)    参考速度 (m/s)，用于计算前视距离
    ~base_speed       (float, default: 1.0)    行驶基础速度 (m/s)
    ~angle_gain       (float, default: -3.5)   转向增益（负值适配 HSP 底盘）
    ~goal_radius      (float, default: 0.05)   到达目标的判定半径 (m)
    ~controller_freq  (int,   default: 20)     控制频率 (Hz)

基于 tianbot/tianracer L1_controller_v2.cpp 改写，保留相同算法逻辑。
"""

import math
import os
import rospy
import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker


class L1Controller(object):
    """L1 路径跟踪控制器"""

    def __init__(self):
        rospy.init_node('l1_controller')

        # ── 参数 ──
        default_name = os.environ.get('TIANRACER_NAME', 'tianracer')
        self.robot_name = rospy.get_param('~robot_name', default_name)
        self.map_frame   = rospy.get_param('~map_frame_id',  self.robot_name + '/map')
        self.odom_frame  = rospy.get_param('~odom_frame_id', self.robot_name + '/odom')

        # 车辆几何
        self.L   = rospy.get_param('~L',   0.26)   # 轴距
        self.lfw = rospy.get_param('~lfw', 0.13)   # 前轮偏移

        # 控制参数
        self.Vcmd         = rospy.get_param('~Vcmd',        1.0)
        self.base_speed   = rospy.get_param('~base_speed',  1.0)
        self.angle_gain   = rospy.get_param('~angle_gain', -3.5)
        self.goal_radius  = rospy.get_param('~goal_radius', 0.05)
        self.ctrl_freq    = rospy.get_param('~controller_freq', 20)

        # ── 状态变量 ──
        self.odom       = Odometry()            # 最新里程计
        self.map_path   = Path()                # 全局路径（map 坐标系）
        self.goal_pos   = Point()               # 目标点（odom 坐标系）
        self.goal_received = False
        self.goal_reached  = False
        self.found_forward_pt = False

        # 前视距离：根据 Vcmd 计算
        self.Lfw = self._calc_l1_distance(self.Vcmd)

        # ── TF ──
        self.tf_listener = tf.TransformListener()

        # ── 订阅 & 发布 ──
        # 使用车机命名空间下的已有话题
        rospy.Subscriber('/tianracer/odometry/filtered',        Odometry,    self._odom_cb,  queue_size=1)
        rospy.Subscriber('/tianracer/move_base/NavfnROS/plan', Path,   self._path_cb,  queue_size=1)
        rospy.Subscriber('/tianracer/move_base_simple/goal',    PoseStamped, self._goal_cb,  queue_size=1)

        self.cmd_pub    = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
        self.marker_pub = rospy.Publisher('car_path',      Marker,         queue_size=10)
    
        # ── 定时器 ──进行周期性控制和目标检查，
        # !进入主程序
        rospy.Timer(rospy.Duration(1.0 / self.ctrl_freq),       self._control_loop)
        rospy.Timer(rospy.Duration(0.5 / self.ctrl_freq),       self._goal_reaching_check)

        rospy.loginfo('L1Controller (Python) started — base_speed=%.2f, angle_gain=%.2f, Lfw=%.2f',
                      self.base_speed, self.angle_gain, self.Lfw)

    # ════════════════════════════════════
    #  回调：只存数据，不做重计算
    # ════════════════════════════════════

    def _odom_cb(self, msg):
        """里程计回调：保存最新里程计"""
        self.odom = msg

    def _path_cb(self, msg):
        """路径回调：保存 move_base 规划的全局路径"""
        self.map_path = msg

    def _goal_cb(self, msg):
        """目标回调：把 RViz 点击的目标从 map 坐标系转到 odom 坐标系"""
        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, self.map_frame, rospy.Time(0), rospy.Duration(1.0))
            odom_goal = self.tf_listener.transformPose(self.odom_frame, msg)
            self.goal_pos = odom_goal.pose.position
            self.goal_received = True
            self.goal_reached  = False
            rospy.loginfo('Goal received: (%.2f, %.2f)', self.goal_pos.x, self.goal_pos.y)
            # 在 RViz 中画目标圆
            self._publish_goal_marker(odom_goal.pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('TF error in goal_cb: %s', e)

    # ════════════════════════════════════
    #  核心算法
    # ════════════════════════════════════

    @staticmethod
    def _yaw_from_pose(pose):
        """从 Pose 的四元数中提取 yaw 角"""
        q = pose.orientation
        # 简化公式，等价于 tf.transformations.euler_from_quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _calc_l1_distance(vcmd):
        """
        根据参考速度计算前视距离 Lfw
        低速用短前视距离（灵敏），高速用长前视距离（稳定）
        """
        if vcmd < 0.5:
            return 0.2
        elif vcmd < 1.34:
            return 0.5
        elif vcmd <= 5.36:
            return vcmd * 2.24 / 3.0
        else:
            return 4.0

    def _is_forward_waypoint(self, waypoint, car_pose):
        """
        判断路径点是否在车的前方
        把世界坐标系下的向量转到车体坐标系，x > 0 即在前方
        """
        dx = waypoint.x - car_pose.position.x
        dy = waypoint.y - car_pose.position.y
        yaw = self._yaw_from_pose(car_pose)
        # 旋转到车体坐标系
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        return local_x > 0

    def _is_far_enough(self, waypoint, car_pos):
        """判断路径点到车的距离是否 >= 前视距离 Lfw"""
        dx = waypoint.x - car_pos.x
        dy = waypoint.y - car_pos.y
        return math.sqrt(dx * dx + dy * dy) >= self.Lfw

    def _find_forward_point(self, car_pose):
        """
        在全局路径上找前视目标点：
        1. 遍历路径点，TF 转到 odom 坐标系
        2. 筛选：在车前方 且 距离 >= Lfw
        3. 取第一个满足条件的点
        4. 如果都不满足，用终点目标作为 fallback
        """
        car_pos = car_pose.position
        forward_pt = Point()
        self.found_forward_pt = False

        if self.goal_reached:
            forward_pt = self.goal_pos
            return forward_pt

        # 遍历路径，找第一个合格的前视点
        for pose_stamped in self.map_path.poses:
            try:
                odom_pose = self.tf_listener.transformPose(
                    self.odom_frame, pose_stamped)
                wp = odom_pose.pose.position

                is_forward = self._is_forward_waypoint(wp, car_pose)
                is_far     = self._is_far_enough(wp, car_pos)

                if is_forward and is_far:
                    forward_pt = wp
                    self.found_forward_pt = True
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        # Fallback：没找到前方点 → 用目标点
        if not self.found_forward_pt:
            dist2goal = self._dist_to_goal()
            if dist2goal > 0.05:
                forward_pt = self.goal_pos
                self.found_forward_pt = True
                rospy.logwarn_throttle(2.0, 'No forward WP, using goal as fallback')

        # 可视化：画车到前视点的连线
        self._publish_tracking_marker(car_pos, forward_pt)

        return forward_pt

    def _compute_eta(self, car_pose, forward_pt):
        """
        计算车头到前视点的方位角 eta（车体坐标系）
        eta > 0 → 目标在左前方
        eta < 0 → 目标在右前方
        """
        yaw = self._yaw_from_pose(car_pose)
        dx = forward_pt.x - car_pose.position.x
        dy = forward_pt.y - car_pose.position.y
        # 转车体坐标系
        local_x =  math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return math.atan2(local_y, local_x)

    def _compute_steering(self, eta):
        """
        L1 转向公式：
        eta 是车头到前视点的方位角，L 是轴距，Lfw 是前视距离，lfw 是前轮偏移
        steering = -atan2(L * sin(eta), Lfw/2 + lfw * cos(eta))
        返回角度（度），还需乘 angle_gain
        """
        steering_rad = -math.atan2(
            self.L * math.sin(eta),
            self.Lfw / 2.0 + self.lfw * math.cos(eta)
        )
        return math.degrees(steering_rad)

    def _dist_to_goal(self):
        """计算车到目标点的距离"""
        car = self.odom.pose.pose.position
        dx = self.goal_pos.x - car.x
        dy = self.goal_pos.y - car.y
        return math.sqrt(dx * dx + dy * dy)

    # ════════════════════════════════════
    #  定时器回调
    # ════════════════════════════════════

    def _goal_reaching_check(self, event):
        """检查是否到达目标"""
        if self.goal_received:
            if self._dist_to_goal() < self.goal_radius:
                self.goal_reached  = True
                self.goal_received = False
                rospy.loginfo('Goal Reached!')

    def _control_loop(self, event):
        """
        主控制循环（20Hz）：
        感知 → 决策 → 执行
        """
        # [初始化] 创建一个新的阿克曼控制指令消息对象
        # 这是一个安全默认值：如果后续逻辑不执行，车辆将保持停止且方向盘回正
        cmd = AckermannDrive()
        
        # [安全默认] 默认速度设为 0.0 (停车)
        cmd.speed = 0.0
        
        # [安全默认] 默认转向角设为 0.0 (直行/回正)
        cmd.steering_angle = 0.0

        # [条件判断] 只有当接收到有效的导航目标点时，才执行跟踪逻辑
        # 如果没有目标点 (goal_received=False)，车辆将保持上述的默认停止状态
        if self.goal_received:
            
            # [感知] 获取当前最新的里程计位姿 (位置 + 姿态)
            # 这是从回调函数 _odom_cb 中实时更新并保存下来的最新数据
            car_pose = self.odom.pose.pose

            # --- 决策阶段 ---
            
            # [步骤 1: 寻找前视点]
            # 调用核心算法函数，在全局路径中寻找符合 L1 条件的“前视点” (Forward Point)
            # 条件通常是：位于车头前方 且 距离 >= 动态前视距离 Lfw
            # 返回结果是一个 geometry_msgs/Point 类型的坐标点
            forward_pt = self._find_forward_point(car_pose)

            # [步骤 2: 计算转向]
            # 双重检查：
            # 1. self.found_forward_pt: 确认上一步确实找到了有效的前视点 (防止路径丢失或异常)
            # 2. not self.goal_reached: 确认尚未到达终点 (防止过冲后继续行驶)
            if self.found_forward_pt and not self.goal_reached:
                
                # [几何计算] 计算方位角 eta (η)
                # 将前视点转换到车体坐标系，计算车头方向与前视点连线之间的夹角
                # eta > 0 表示点在左前方，eta < 0 表示点在右前方
                eta = self._compute_eta(car_pose, forward_pt)
                
                # [L1 核心公式] 根据方位角 eta 计算理论转向角
                # 使用 L1 公式：delta = -atan2(L*sin(eta), Lfw/2 + lfw*cos(eta))
                # 返回结果是角度制 (degrees)，eta 是车头到前视点的方位角，L 是轴距，Lfw 是前视距离，lfw 是前轮偏移
                steering_deg = self._compute_steering(eta)
                
                # [增益修正] 应用转向增益系数
                # angle_gain 用于适配具体底盘的转向特性 (如转向比、方向正反)
                # 例如：如果底盘向左转需要负角度，这里 gain 可能是负数
                cmd.steering_angle = steering_deg * self.angle_gain
                
                # [速度设定] 设定行驶速度
                # L1 控制器通常采用恒速策略 (Base Speed)，速度大小不随曲率变化

                cmd.speed = self.base_speed

        # [执行] 发布最终的控制指令到 ROS 话题
        # 无论是否找到目标点，都会发布消息：
        # - 如果正常跟踪：发布计算出的速度和角度
        # - 如果无目标/已到达/未找到点：发布默认的 0 速度和 0 角度 (安全停车)
        self.cmd_pub.publish(cmd)

    # ════════════════════════════════════
    #  RViz 可视化
    # ════════════════════════════════════

    def _publish_goal_marker(self, goal_pose):
        """在目标位置画黄色圆柱"""
        m = Marker()
        m.header.frame_id = self.odom_frame
        m.header.stamp = rospy.Time.now()
        m.ns = 'l1_controller'
        m.id = 2
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose = goal_pose
        m.scale.x = self.goal_radius
        m.scale.y = self.goal_radius
        m.scale.z = 0.1
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.a = 0.5
        self.marker_pub.publish(m)

    def _publish_tracking_marker(self, car_pos, forward_pt):
        """画车到前视点的绿点 + 蓝线"""
        now = rospy.Time.now()

        # 绿色端点
        pts = Marker()
        pts.header.frame_id = self.odom_frame
        pts.header.stamp = now
        pts.ns = 'l1_controller'
        pts.id = 0
        pts.type = Marker.POINTS
        pts.action = Marker.ADD
        pts.scale.x = 0.2
        pts.scale.y = 0.2
        pts.color.g = 1.0
        pts.color.a = 1.0
        pts.pose.orientation.w = 1.0

        # 蓝色连线
        line = Marker()
        line.header.frame_id = self.odom_frame
        line.header.stamp = now
        line.ns = 'l1_controller'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.1
        line.color.b = 1.0
        line.color.a = 1.0
        line.pose.orientation.w = 1.0

        if self.found_forward_pt and not self.goal_reached:
            pts.points = [car_pos, forward_pt]
            line.points = [car_pos, forward_pt]

        self.marker_pub.publish(pts)
        self.marker_pub.publish(line)


def main():
    controller = L1Controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
