#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 Python 示例：不调用 spin() 的订阅者
- 演示内容：订阅 /chatter 但不调用 rospy.spin()，仅短暂等待后退出；显示不保持常驻会停止接收后续消息。
- 提示：在 rospy 中，回调由内部线程触发；不调用 spin() 时，脚本一旦结束，进程退出就无法再接收消息。
"""

import rospy
from std_msgs.msg import String


def on_message_callback(msg):
    rospy.loginfo('Received (no spin): %s', msg.data)


def main():
    # 初始化节点，节点名为 chatter_sub_nospin
    rospy.init_node('chatter_sub_nospin')

    # 订阅 /chatter 话题
    rospy.Subscriber('/chatter', String, on_message_callback)

    rospy.loginfo('Subscriber without spin(): will exit after 3 seconds...')

    # 不调用 spin()；仅睡眠 3 秒模拟短暂存活，随后退出
    rospy.sleep(3.0)
    rospy.loginfo('Exiting without spin(). Further messages will not be received.')


if __name__ == '__main__':
    main()
