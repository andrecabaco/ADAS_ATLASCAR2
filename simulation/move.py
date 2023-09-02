#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def publish_twist():
    rospy.init_node('twist_publisher', anonymous=True)
    pub = rospy.Publisher('/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # First phase: Linear velocity for 10 seconds
    twist_msg = Twist()
    twist_msg.linear.x = 0.1
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = -1.0

    start_time = time.time()
    while time.time() - start_time < 2.5:
        pub.publish(twist_msg)
        rate.sleep()

    # Second phase: Angular velocity for 10 seconds
    twist_msg.angular.z = 0.0
    twist_msg.linear.x = 1.0

    start_time = time.time()
    while time.time() - start_time < 126.5:
        pub.publish(twist_msg)
        rate.sleep()

    # Second phase: Angular velocity for 10 seconds
    twist_msg.angular.z = -1.0
    twist_msg.linear.x = 1.0

    start_time = time.time()
    while time.time() - start_time < 6.3:
        pub.publish(twist_msg)
        rate.sleep()


    # Third phase: Stop
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0

    while not rospy.is_shutdown():
        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_twist()
    except rospy.ROSInterruptException:
        pass
