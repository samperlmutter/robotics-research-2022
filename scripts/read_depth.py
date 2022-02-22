#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2

cy = 239.5
cx = 319.5
fx = 570.3422
fy = 319.5


def compute_world_pos(x, y, depth):
    return ((x - cx) * depth / fx,
            (y - cy) * depth / fy,
            depth,
            1)


def main():
    rospy.init_node('contact_detector')
    img = rospy.wait_for_message('/camera/depth/image_raw', Image)
    bridge = CvBridge()
    cv_img = np.array(bridge.imgmsg_to_cv2(img, desired_encoding='32FC1') / 1000)
    rospy.loginfo(cv_img.shape)

    img_pts = np.empty(cv_img.shape, dtype=object)
    for y in range(len(img_pts)):
        for x in range(len(img_pts[0])):
            img_pts[y][x] = compute_world_pos(x, y, cv_img[y][x])

    rospy.loginfo(img_pts[100][200])

    # cv_img_gray = np.array((cv_img / max(map(max, cv_img))) * 255, dtype=np.uint8)
    # cv2.imshow('depth', cv_img_gray)
    # cv2.waitKey(0)


if __name__ == '__main__':
    main()
