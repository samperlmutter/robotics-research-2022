#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import util
from ee_pose_detection.msg import DepthPixel


def main():
    rospy.init_node('contact_detector')
    pixel = rospy.wait_for_message('/contact/pose/pixel', DepthPixel)
    img = rospy.wait_for_message('/camera/depth/image_raw', Image)
    bridge = CvBridge()
    cv_img = np.array(bridge.imgmsg_to_cv2(img, desired_encoding='32FC1') / 1000)
    rospy.loginfo(pixel)

    img_pts = np.empty(cv_img.shape, dtype=object)
    for y in range(len(img_pts)):
        for x in range(len(img_pts[0])):
            img_pts[y][x] = util.pixel_to_pose(x, y, cv_img[y][x])

    cv_img_gray = np.array((cv_img / max(map(max, cv_img))) * 255, dtype=np.uint8)
    cv_img_gray[int(pixel.x)][int(pixel.y)] = 255  # indicate pixel in grayscale img

    cv2.imshow('depth', cv_img_gray)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
