#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2


def main():
  rospy.init_node('contact_detector')
  img = rospy.wait_for_message('/camera/depth/image_raw', Image)
  bridge = CvBridge()
  cv_img = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
  rospy.loginfo(len(img.data))
  data = []
  for i in range(0, len(img.data[:5]), 2):
      rospy.loginfo((str.encode(img.data[i]) << 8) | str.encode(img.data[i+1]))
    #   data.append((bytearray(img.data[i]) << 8) | bytearray(img.data[i+1]))
  
  rospy.loginfo(data)


if __name__ == '__main__':
  main()