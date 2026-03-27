#! /usr/bin/env python
# 创建者：陈宇轩
# 修改者：田博
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

FILTER_VALUE = 10.0 #无效值替代
last_scan_time = None #记录上次收到激光数据的时间
cmd_speed = 0.0 # 期望速度
cmd_steer = 0.0 # 期望转向角

# 获取激光数据中指定角度范围的最小值
def get_range(data, angle, deg=True):
    if deg:
        # 将角度转换为弧度
        angle = np.deg2rad(angle)
        #索引 = (目标角度 - 起始角度) / 分辨率
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis


def wall_following_callback(data):
    """
    实现简易巡墙算法，参考：
    https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
    """
    # 两束激光之间的夹角
    THETA = np.pi / 180 * 60
    # 目标与墙的距离
    TARGET_DIS = 1
    # 车辆前视投影距离
    LOOK_AHEAD_DIS = 5
    # 转向控制的比例系数
    P = 0.5

    # 变量命名沿用上述 PDF
    # 获取激光数据中指定角度范围的最小值
    #a为-60度，b为-90度（相对于车头）
    b = get_range(data, -90)
    a = get_range(data, -90 + np.rad2deg(THETA))#转成角度制
    # print(f"a{a:1.1f} b{b:1.1f}")
    # 算墙的朝向角 alpha 和与墙的距离 projected_dis
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    # 车到墙的真实距离
    AB = b * np.cos(alpha)
    #未来车与墙的距离（投影距离）
    projected_dis = AB + LOOK_AHEAD_DIS * np.sin(alpha)
    error = TARGET_DIS - projected_dis
    steering_angle = P * error

    front_dis = get_range(data, 0)
    # 速度可设置为 0.5 到 3.5 m/s，默认 3
    speed = 1 
    angle_filter = steering_angle
    
    global cmd_speed, cmd_steer, last_scan_time
    #速度控制：以设定速度前进
    cmd_speed = speed
    #转向控制：根据误差调整转向角
    cmd_steer = steering_angle
    last_scan_time = rospy.Time.now()

def timer_callback(event):
    global last_scan_time, cmd_speed, cmd_steer
    
    # 超时保护：0.5 秒内未收到激光数据则停止
    if last_scan_time is None or (rospy.Time.now() - last_scan_time).to_sec() > 0.5:
        cmd_speed = 0.0
        cmd_steer = 0.0
        rospy.logwarn_throttle(2.0, "Time out...")

    msg = AckermannDrive()
    msg.speed = cmd_speed
    msg.steering_angle = cmd_steer
    drive_pub.publish(msg)

if __name__ == '__main__': 
    try:
        rospy.init_node("wall_following")
        
        # 初始化时间，避免立即超时
        last_scan_time = rospy.Time.now()
        
        scan_sub = rospy.Subscriber('/tianracer/scan', LaserScan, wall_following_callback)
        drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)
        
        # 添加定时器，保持控制循环频率（30Hz）
        rospy.Timer(rospy.Duration(1.0/30.0), timer_callback)
        
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass