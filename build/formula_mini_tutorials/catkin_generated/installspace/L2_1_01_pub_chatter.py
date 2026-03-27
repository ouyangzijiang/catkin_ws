#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 Python 示例：发布者（Publisher）
- 演示内容：Topic 发布（/chatter）、消息驱动范式、发布频率控制
- 相关概念：发布者、消息类型 std_msgs/String、rospy.Rate
- 支持参数：~group_id（组号，默认 'X'）、~rate（发布频率Hz，默认 1.0）

运行前确保 roscore 已启动；推荐按 README 手动运行发布者与订阅者。
"""

import rospy
from std_msgs.msg import String


def main():
    # 初始化节点，节点名为 chatter_pub
    rospy.init_node('chatter_pub')

    # 获取私有参数：组号与发布频率
    group_id = rospy.get_param('~group_id', 'X')
    rate_hz = float(rospy.get_param('~rate', 1.0))

    # 创建发布者：话题 /chatter，消息类型 String
    pub = rospy.Publisher('/chatter', String, queue_size=10)

    rate = rospy.Rate(rate_hz)
    count = 0
    rospy.loginfo('Publisher started. topic=/chatter, group_id=%s, rate=%.2fHz', group_id, rate_hz)

    # 持续循环发布消息，直到节点关闭
    while not rospy.is_shutdown():
        # 构造字符串消息：包含组号与递增序号
        msg_text = 'Hello from Group{0} #{1}'.format(group_id, count)

        # 发布到 /chatter
        pub.publish(msg_text)

        # 使用 ROS 日志输出已发布内容（更符合 ROS 习惯）
        rospy.loginfo('Published: %s', msg_text)

        count += 1
        rate.sleep()


if __name__ == '__main__':
    main()
