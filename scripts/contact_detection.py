#!/usr/bin/env python

import rospy
import struct
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2

pc_pub = rospy.Publisher('/contact/cloud', PointCloud2, queue_size=10)


def near_pose(pose, a, b, c):
  return ((a - pose.position.x) ** 2) + ((b - pose.position.y) ** 2) + ((c - pose.position.z) ** 2) <= 2 ** 2


def contact_cb(pose):
  points = point_cloud_cb(rospy.wait_for_message('/camera/depth/points', PointCloud2))
  close_points = filter(lambda p: near_pose(pose.pose, p[0], p[1], p[2]), points)

  h = Header()
  h.stamp = rospy.Time.now()
  h.frame_id = 'camera_rgb_optical_frame'
  
  pc = pc2.create_cloud_xyz32(h, close_points)

  pc_pub.publish(pc)


def point_cloud_cb(point_cloud):
  points = []
  points_gen = pc2.read_points(point_cloud, skip_nans=True)
  for p in points_gen:
    points.append(p)

  return points
  

def main():
  rospy.init_node('contact_detector')
  rospy.Subscriber('/contact/pose', PoseStamped, contact_cb)
  rospy.Subscriber('/camera/depth/points', PointCloud2, point_cloud_cb)

  rospy.spin()

if __name__ == '__main__':
  main()
