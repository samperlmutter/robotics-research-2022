#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import util
from ee_pose_detection.msg import DepthPixel

pose_img_pub = rospy.Publisher('/contact/image', Image, queue_size=10)
depth_threshold = 0.1


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

    pixel_pose = util.pixel_to_pose(pixel.x, pixel.y, pixel.depth)
    pixel_n = util.pose_to_pixel(pixel_pose.position.x, pixel_pose.position.y + depth_threshold, pixel_pose.position.z)
    pixel_s = util.pose_to_pixel(pixel_pose.position.x, pixel_pose.position.y - depth_threshold, pixel_pose.position.z)
    pixel_e = util.pose_to_pixel(pixel_pose.position.x - depth_threshold, pixel_pose.position.y, pixel_pose.position.z)
    pixel_w = util.pose_to_pixel(pixel_pose.position.x + depth_threshold, pixel_pose.position.y, pixel_pose.position.z)
    pixels = [pixel_n, pixel_e, pixel_s, pixel_w]
    for p in pixels:
        rospy.loginfo(p)
        rospy.loginfo(abs(pixel.depth - p.depth))
        if abs(pixel.depth - p.depth) > depth_threshold:
            pixels.remove(p)

    cv_img_gray = np.array((cv_d_img / max(map(max, cv_d_img))) * 255, dtype=np.uint8)

    def highlight_pixel(p, color):
        if p.x in range(0, len(cv_img_gray[0])) and p.y in range(0, len(cv_img_gray)):
            cv_img_gray[int(p.y)][int(p.x)] = 255
            cv_c_img[int(p.y)][int(p.x)] = color
        else:
            rospy.loginfo(p)

    highlight_pixel(pixel, [255, 255, 255])
    highlight_pixel(pixel_n, [0, 0, 255])
    highlight_pixel(pixel_s, [0, 255, 0])
    highlight_pixel(pixel_e, [255, 0, 0])
    highlight_pixel(pixel_w, [0, 255, 255])

    pts = np.array([[pixel_n.x, pixel_n.y], [pixel_e.x, pixel_e.y], [pixel_s.x, pixel_s.y], [pixel_w.x, pixel_w.y]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(cv_c_img, [pts], True, (0, 0, 255))

    # d_img = bridge.cv2_to_imgmsg(cv_img_gray, encoding='mono8')
    c_img = bridge.cv2_to_imgmsg(cv_c_img, encoding='bgr8')
    pose_img_pub.publish(c_img)


if __name__ == '__main__':
    rospy.init_node('contact_detector')
    pose_sub = rospy.Subscriber('/contact/pose/pixel', DepthPixel, callback=pixel_cb)
    rospy.spin()
