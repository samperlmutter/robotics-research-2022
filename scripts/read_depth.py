#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import util
from ee_pose_detection.msg import DepthPixel
import math

pose_img_pub = rospy.Publisher('/contact/image', Image, queue_size=10)
depth_threshold = 0.2
search_area = 0.1
seg_verts = 20


def pixel_cb(pixel):
    d_img = rospy.wait_for_message('/camera/depth/image_raw', Image)
    c_img = rospy.wait_for_message('/camera/rgb/image_raw', Image)
    bridge = CvBridge()
    cv_d_img = np.array(bridge.imgmsg_to_cv2(d_img, desired_encoding='32FC1') / 1000)
    cv_c_img = np.array(bridge.imgmsg_to_cv2(c_img, desired_encoding='bgr8'))

    if pixel.x >= len(cv_d_img[0]) or pixel.y >= len(cv_d_img):
        return

    img_pts = np.empty(cv_d_img.shape, dtype=object)
    for y in range(len(img_pts)):
        for x in range(len(img_pts[0])):
            img_pts[y][x] = util.pixel_to_pose(x, y, cv_d_img[y][x])
    rospy.loginfo(cv_d_img[pixel.y][pixel.x])
    pixel_pose = util.pixel_to_pose(pixel.x, pixel.y, pixel.depth)

    pixels = []
    for t in range(0, 360, 360//seg_verts):
        x = pixel_pose.position.x + math.cos(math.radians(t)) * search_area
        y = pixel_pose.position.y + math.sin(math.radians(t)) * search_area
        p = util.pose_to_pixel(x, y, pixel_pose.position.z)
        p.depth = cv_d_img[p.y][p.x]
        cv_c_img[int(p.y)][int(p.x)] = [255, 0, 0]
        if abs(pixel.depth - p.depth) <= depth_threshold:
            pixels.append(p)
    cv_img_gray = np.array((cv_d_img / max(map(max, cv_d_img))) * 255, dtype=np.uint8)

    def highlight_pixel(p, color):
        if p.x in range(0, len(cv_img_gray[0])) and p.y in range(0, len(cv_img_gray)):
            cv_img_gray[int(p.y)][int(p.x)] = 255
            cv_c_img[int(p.y)][int(p.x)] = color
        else:
            rospy.loginfo(p)

    pts = np.array([[p.x, p.y] for p in pixels], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.fillPoly(cv_c_img, [pts], color=(0, 0, 255))

    highlight_pixel(pixel, [0, 255, 0])
    for p in pixels:
        highlight_pixel(p, [255, 0, 0])

    c_img = bridge.cv2_to_imgmsg(cv_c_img, encoding='bgr8')
    pose_img_pub.publish(c_img)


if __name__ == '__main__':
    rospy.init_node('contact_detector')
    pose_sub = rospy.Subscriber('/contact/pose/pixel', DepthPixel, callback=pixel_cb)
    rospy.spin()
