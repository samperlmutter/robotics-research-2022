#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import util

if __name__ == '__main__':
    rospy.init_node('show_pixel', anonymous=True)
    x = int(sys.argv[1])
    y = int(sys.argv[2])
    c_img = rospy.wait_for_message('/camera/rgb/image_raw', Image)
    d_img = rospy.wait_for_message('/camera/depth/image_raw', Image)
    bridge = CvBridge()
    cv_c_img = np.array(bridge.imgmsg_to_cv2(c_img, desired_encoding='bgr8'))
    cv_d_img = np.array(bridge.imgmsg_to_cv2(d_img, desired_encoding='32FC1') / 1000)

    for i in range(-1, 2):
        for j in range(-1, 2):
            cv_c_img[y + i][x + j] = [0, 0, 255]
    rospy.loginfo(f'pixel: {x}, {y}, {cv_d_img[y][x]}')
    pose = util.pixel_to_pose(x, y, cv_d_img[y][x])
    rospy.loginfo(f'pose: {pose.position.x}, {pose.position.y}, {pose.position.z}')

    cv2.imshow('img', cv_c_img)
    cv2.waitKey(0)
