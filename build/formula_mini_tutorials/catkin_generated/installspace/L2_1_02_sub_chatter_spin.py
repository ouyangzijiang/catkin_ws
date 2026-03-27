#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 Python 示例：订阅者（Subscriber）+ 回调（Callback）+ spin()
- 演示内容：订阅 /chatter，收到消息即触发回调打印；使用 rospy.spin() 保持进程常驻。
- 相关概念：回调函数每次有新消息到达就执行一次；spin() 让节点持续等待并处理消息。
"""

import rospy
from std_msgs.msg import String


def on_message_callback(msg):
    """订阅回调：每当有新消息到达，就会自动被调用一次。"""
    # 在终端打印收到的内容
    rospy.loginfo('Received: %s', msg.data)


def main():
    # 初始化节点，节点名为 chatter_sub_spin
    rospy.init_node('chatter_sub_spin')

    # 订阅 /chatter 话题，消息类型为 String，注册回调函数
    rospy.Subscriber('/chatter', String, on_message_callback)

    rospy.loginfo('Subscriber with spin() started, waiting for messages...')

    # spin() 进入消息循环：让节点持续运行并处理到来的消息
    rospy.spin()


if __name__ == '__main__':
    main()
