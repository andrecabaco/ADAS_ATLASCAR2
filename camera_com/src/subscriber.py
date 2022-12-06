#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2



def msgReceivedCallback(msg):
    rospy.loginfo("I received an image")

    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    cv2.imshow("Subscriber", cv_image)
    cv2.waitKey(30)

    
def main():




    # Initialization of a ros node
    rospy.init_node('camera_subscriber', anonymous=True)

    # Init the subscriber
    rospy.Subscriber('camera_communication', Image, msgReceivedCallback)



    # ------------------------------------
    # Execution 
    # ------------------------------------
    rospy.spin()

    # ------------------------------------
    # Termination 
    # ------------------------------------

if __name__ == '__main__':
    main()