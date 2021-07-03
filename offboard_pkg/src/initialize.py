#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/aruco_detect",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/iris_fpv_cam/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    cv_image = cv2.imread('/home/ruei/catkin_ws/src/offboard_pkg/marker/1.png',1)

    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
