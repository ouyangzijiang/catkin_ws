#!/usr/bin/env python3
# coding: utf-8

"""
Pure Pursuit 路径跟踪节点 (ROS)

功能：
- 从 CSV 文件加载路径点
- 基于 Pure Pursuit 算法计算前视点
- 生成 AckermannDrive 控制指令
- 可视化路径和目标点（Marker）
"""

import csv
import math
import os

import rospy
import tf2_ros
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

# 尝试导入 rospkg 用于获取 ROS 包路径
try:
    import rospkg
except ImportError:
    rospkg = None

# ------------------------- 工具函数 -------------------------

def normalize_frame_id(frame_id):
    """
    去掉 frame_id 开头的 '/'，统一 TF 框架命名
    """
    return frame_id.lstrip("/")

def yaw_from_quaternion(q):
    """
    从四元数计算偏航角 (yaw)
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_to_pi(angle):
    """
    将角度归一化到 [-pi, pi]
    """
    return math.atan2(math.sin(angle), math.cos(angle))

# ------------------------- Pure Pursuit 节点类 -------------------------

class PurePursuitNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("l6_4_01_pure_pursuit", anonymous=False)

        # ------------------- 参数读取 -------------------
        self.robot_name = rospy.get_param("~robot_name", "tianracer")

        # 里程计、控制话题、可视化 Marker 话题
        self.odom_topic = rospy.get_param("~odom_topic", f"/{self.robot_name}/odom")
        self.drive_topic = rospy.get_param("~drive_topic", f"/{self.robot_name}/ackermann_cmd")
        self.path_marker_topic = rospy.get_param("~path_marker_topic", f"/{self.robot_name}/pure_pursuit/path_marker")
        self.target_marker_topic = rospy.get_param("~target_marker_topic", f"/{self.robot_name}/pure_pursuit/target_marker")
        self.target_marker_odom_topic = rospy.get_param(
            "~target_marker_odom_topic", f"/{self.robot_name}/pure_pursuit/target_marker_odom"
        )

        # TF 框架
        marker_frame = rospy.get_param("~marker_frame", f"{self.robot_name}/map")
        self.marker_frame = normalize_frame_id(marker_frame)
        odom_frame = rospy.get_param("~odom_frame", f"{self.robot_name}/odom")
        self.odom_frame = normalize_frame_id(odom_frame)

        # Pure Pursuit 参数
        self.lookahead_dist = rospy.get_param("~lookahead_dist", 0.8)  # 前视距离
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.18) # 到达目标阈值
        self.wheelbase = rospy.get_param("~wheelbase", 0.33)           # 轴距
        self.max_steer_deg = rospy.get_param("~max_steer_deg", 25.0)   # 最大转向角
        self.steer_sign = rospy.get_param("~steer_sign", 1.0)          # 转向方向

        # 速度设置
        self.speed_straight = rospy.get_param("~speed_straight", 0.375)
        self.speed_curve = rospy.get_param("~speed_curve", 0.2625)
        self.speed_tight = rospy.get_param("~speed_tight", 0.15)
        self.stop_speed = rospy.get_param("~stop_speed", 0.0)

        # ------------------- 加载路径点 -------------------
        self.path_points = self._load_waypoints()
        if len(self.path_points) < 2:
            raise rospy.ROSInitException("Waypoint count must be >= 2 for pure pursuit.")

        # ------------------- TF -------------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_target_idx = 0

        # ------------------- 发布器 -------------------
        self.pub_drive = rospy.Publisher(self.drive_topic, AckermannDrive, queue_size=10)
        self.pub_path_marker = rospy.Publisher(self.path_marker_topic, Marker, queue_size=1, latch=True)
        self.pub_target_marker = rospy.Publisher(self.target_marker_topic, Marker, queue_size=10)
        self.pub_target_marker_odom = rospy.Publisher(self.target_marker_odom_topic, Marker, queue_size=10)

        # ------------------- 订阅里程计 -------------------
        #!进入主进程
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

        # ------------------- 可视化路径 -------------------
        self.path_marker = self._build_path_marker()
        self.pub_path_marker.publish(self.path_marker)

        rospy.loginfo("[PurePursuit] Ready. odom=%s drive=%s marker_frame=%s odom_frame=%s waypoints=%d",
                      self.odom_topic, self.drive_topic, self.marker_frame, self.odom_frame, len(self.path_points))

    # ------------------- 路径点文件 -------------------
    def _default_waypoint_file(self):
        """
        默认路径点文件路径
        优先从 ROS 包中读取 formula_mini_tutorials/data/waypoints.csv
        否则使用脚本目录下 data.csv
        """
        if rospkg is not None:
            try:
                pkg_path = rospkg.RosPack().get_path("formula_mini_tutorials")
                return os.path.join(pkg_path, "data", "waypoints.csv")
            except Exception:
                pass

        script_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(script_dir, "data.csv")

    def _load_waypoints(self):
        """
        从 CSV 文件加载路径点
        """
        waypoint_file = rospy.get_param("~waypoint_file", self._default_waypoint_file())
        if not os.path.isabs(waypoint_file):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            waypoint_file = os.path.join(script_dir, waypoint_file)

        if not os.path.exists(waypoint_file):
            raise rospy.ROSInitException(f"Waypoint file not found: {waypoint_file}")

        pts = []
        with open(waypoint_file, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or len(row) < 2:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                    pts.append((x, y))
                except ValueError:
                    continue

        rospy.loginfo("[PurePursuit] Loaded %d waypoints from %s", len(pts), waypoint_file)
        return pts

    # ------------------- 可视化路径 -------------------
    def _build_path_marker(self):
        """
        构建路径 Marker 用于 RViz 可视化
        """
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit_path"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.points = []

        for x, y in self.path_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        return marker

    def _publish_target_marker(self, x, y):
        """
        发布目标点 Marker（map 坐标系）
        """
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        self.pub_target_marker.publish(marker)

    def _publish_target_marker_odom(self, x, y):
        """
        发布目标点 Marker（odom 坐标系）
        """
        marker = Marker()
        marker.header.frame_id = self.odom_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit_target_odom"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2
        self.pub_target_marker_odom.publish(marker)

    # ------------------- 坐标转换 -------------------
    def _transform_point_2d(self, x, y, transform):
        """
        将点 (x, y) 从一个坐标系转换到另一个坐标系
        transform: geometry_msgs/TransformStamped
        """
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        yaw_t = yaw_from_quaternion(transform.transform.rotation)

        cos_t = math.cos(yaw_t)
        sin_t = math.sin(yaw_t)
        x_new = tx + cos_t * x - sin_t * y
        y_new = ty + sin_t * x + cos_t * y
        return x_new, y_new

    def _path_points_in_odom(self):
        """
        将路径点从 map 坐标系转换到 odom 坐标系
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.marker_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

        pts_odom = []
        for px, py in self.path_points:
            ox, oy = self._transform_point_2d(px, py, trans)
            pts_odom.append((ox, oy))
        return pts_odom

    # ------------------- Pure Pursuit 核心计算 -------------------
    def _find_closest_idx(self, x, y, points):
        """
        找到距离当前位置最近的路径点索引
        """
        best_idx = 0
        best_d2 = float("inf")
        for i, (px, py) in enumerate(points):
            dx = px - x
            dy = py - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        return best_idx

    def _find_lookahead_idx(self, x, y, yaw, start_idx, points):
        """
        根据前视距离找到目标点索引
        """
        n = len(points)
        idx = start_idx
        lookahead_dist2 = self.lookahead_dist * self.lookahead_dist

        for _ in range(n):
            tx, ty = points[idx]
            dist2 = self._compute_points_distance_squared((x, y), (tx, ty))
            rel_x, _ = self._compute_relative_xy_offset(x, y, yaw, tx, ty)
            if dist2 >= lookahead_dist2 and rel_x > 0.0:
                return idx
            idx = (idx + 1) % n

        return start_idx

    def _goal_reached_or_passed(self, x, y, yaw, points):
        """
        判断是否到达终点或已超过终点
        """
        gx, gy = points[-1]
        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)

        if dist <= self.goal_tolerance:
            return True

        x_goal_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        if x_goal_body < 0.0 and dist <= self.goal_tolerance * 1.5:
            return True

        return False

    def _speed_from_steer(self, steer):
        """
        根据转向角选择速度
        """
        abs_deg = abs(math.degrees(steer))
        if abs_deg > 18.0:
            return self.speed_tight
        if abs_deg > 8.0:
            return self.speed_curve
        return self.speed_straight

    def _compute_points_distance_squared(self, point1, point2):
        """
        计算两点距离平方
        """
        dx = point1[0] - point2[0]
        dy = point1[1] - point2[1]
        return (dx * dx) + (dy * dy)

    def _compute_relative_xy_offset(self, current_x, current_y, current_yaw, target_x, target_y):
        """
        将目标点转换到机器人坐标系下,从odom坐标系转换到机器人坐标系(base_link)，返回 (x_body, y_body)
        """
        diff_x = target_x - current_x
        diff_y = target_y - current_y
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
         # x_body: 目标点在车头方向的投影距离
        relative_x = (cos_yaw * diff_x) + (sin_yaw * diff_y)
        # y_body: 目标点在车身左侧方向的投影距离 (横向偏差)
        relative_y = (-sin_yaw * diff_x) + (cos_yaw * diff_y)
        return relative_x, relative_y

    def _compute_steering_rad(self, current_x, current_y, current_yaw, target_x, target_y):
        """
        计算 Pure Pursuit 转向角 (rad)
        【几何原理】
        假设车辆沿圆弧行驶经过前视点，该圆弧曲率 kappa 定义为:
        kappa = 2 * y_body / (x_body^2 + y_body^2)
              = 2 * y_body / L_d^2
        其中 (x_body, y_body) 是目标点在车辆坐标系下的坐标，L_d 是前视距离。
        【运动学模型】
        根据阿克曼几何: tan(delta) = L * kappa
        故转向角 delta = atan(L * kappa)
        【最终公式】
        delta = atan( Wheelbase * (2 * y_body) / (x_body^2 + y_body^2) )

        """
        # 1. 计算分母: L_d^2 (当前点到目标点的距离平方)
        # 对应公式中的: x_body^2 + y_body^2
        denominator = self._compute_points_distance_squared((current_x, current_y), (target_x, target_y))
         # 2. 获取分子所需部分: y_body (目标点在车体坐标系下的横向偏差)
        # x_body 在此公式中被包含在 denominator 里了，不需要单独取出
        _, numerator = self._compute_relative_xy_offset(current_x, current_y, current_yaw, target_x, target_y)
        # 3. 防止除以零
        epsilon = 1e-4
         # 4. 计算曲率 kappa = (2 * y_body) / L_d^2
        curvature = ((2.0 * numerator) / denominator) if denominator > epsilon else 0.0
        # 5. 计算转向角 delta = atan(Wheelbase * kappa)
        # self.wheelbase 即公式中的 L
        return math.atan(curvature * self.wheelbase)

    def _publish_drive(self, speed, steering):
        """
        发布 Ackermann 控制指令
        """
        max_steer = math.radians(self.max_steer_deg)
        steering = max(-max_steer, min(max_steer, steering))

        msg = AckermannDrive()
        msg.speed = speed
        msg.steering_angle = steering
        self.pub_drive.publish(msg)

    # ------------------- 回调函数 -------------------
    def odom_callback(self, msg):
        """
        里程计回调函数，执行 Pure Pursuit 核心逻辑
        """
        path_points_odom = self._path_points_in_odom()
        if not path_points_odom:
            rospy.logwarn_throttle(1.0, "[PurePursuit] Waiting TF %s -> %s", self.marker_frame, self.odom_frame)
            self._publish_drive(self.stop_speed, 0.0)
            return

        # ------------------- 1. 获取当前机器人位姿 -------------------
        # 从里程计消息中提取机器人当前位置 (x, y) 和高度 (z)
        # 这些坐标通常位于 odom 坐标系下
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # 将四元数表示的姿态转换为偏航角 (yaw)，即机器人的朝向角度 (-pi 到 pi)
        # 这是计算相对方位角的关键
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # ------------------- 2. 终点检测 -------------------
        # 调用辅助函数判断机器人是否已经到达或越过了路径的最后一个点
        # 参数包括当前位姿 (x, y, yaw) 和转换到 odom 坐标系下的所有路径点
        if self._goal_reached_or_passed(x, y, yaw, path_points_odom):
            # 如果已到达终点，发布停止指令：速度为 stop_speed (通常为0)，转向角为 0
            self._publish_drive(self.stop_speed, 0.0)
            # 直接返回，不再执行后续的跟踪计算
            return

        # ------------------- 3. 寻找前视点 (Lookahead Point) -------------------
        # 步骤 A: 在所有路径点中，找到距离当前机器人位置 (x, y) 欧氏距离最近的那个点的索引
        # 这作为搜索前视点的起始参考位置，避免从头遍历整个路径
        closest_idx = self._find_closest_idx(x, y, path_points_odom)
        
        # 步骤 B: 从最近点开始向后搜索，找到第一个满足以下条件的点作为“前视点” (Target Point)：
        # 1. 距离机器人的距离 >= 设定的前视距离 (lookahead_dist)
        # 2. 位于机器人车头前方 (相对 x 坐标 > 0)
        # 该函数返回满足条件的路径点在列表中的索引
        target_idx = self._find_lookahead_idx(x, y, yaw, closest_idx, path_points_odom)
        
        # 更新类成员变量，记录当前正在追踪的目标点索引，可用于调试或外部查询
        self.current_target_idx = target_idx

        # ------------------- 4. 可视化反馈 (RViz) -------------------
        # 获取目标点在原始 map 坐标系下的坐标 (用于在全局地图中显示)
        tx_map, ty_map = self.path_points[target_idx]
        # 发布一个红色球体 Marker 到 map 坐标系的话题，在 RViz 中显示全局目标点
        self._publish_target_marker(tx_map, ty_map)
        
        # 获取目标点在 odom 坐标系下的坐标 (用于与机器人相对位置显示)
        tx_odom, ty_odom = path_points_odom[target_idx]
        # 发布一个绿色球体 Marker 到 odom 坐标系的话题，在 RViz 中显示相对目标点
        self._publish_target_marker_odom(tx_odom, ty_odom)

        # ------------------- 5. 计算控制量 -------------------
        # 核心算法：根据当前位姿 (x, y, yaw) 和目标点位置 (tx_odom, ty_odom)
        # 利用 Pure Pursuit 几何公式计算所需的转向角 (弧度制)
        # 公式逻辑：delta = atan(2 * L * y_body / (x_body^2 + y_body^2))
        #x_body 和 y_body 指的是目标点（前视点）相对于机器人车体坐标系（Body Frame）的局部坐标。
        steering = self._compute_steering_rad(x, y, yaw, tx_odom, ty_odom)
        
        # 策略调整：根据计算出的转向角大小动态调整行驶速度
        # 转向角越大 (弯越急)，速度越低，以保证稳定性和安全性
        speed = self._speed_from_steer(steering)
        
        # 发布最终控制指令
        # speed: 计算出的线速度
        # self.steer_sign * steering: 应用转向方向符号修正 (适配不同底盘的转向正反向定义) 后发布
        self._publish_drive(speed, self.steer_sign * steering)

# ------------------------- 程序入口 -------------------------

if __name__ == "__main__":
    try:
        PurePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass