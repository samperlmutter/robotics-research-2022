#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
# import ros_numpy
# import pcl
import numpy as np

pc_pub = rospy.Publisher('/contact/cloud', PointCloud2, queue_size=10)


def near_pose(pose, a, b, c):
  marker = Marker()
  marker.type = 3
  marker.pose = pose
  marker.scale.x = 1
  marker.scale.y = 1
  marker.scale.z = 1
  marker.header.frame_id = 'base_link'
  marker.header.stamp = rospy.Time()
  # point_pub.publish(marker)
  return ((pose.position.x - a) ** 2) + ((pose.position.y - b) ** 2) + ((pose.position.z - c) ** 2) <= 1 ** 2


def contact_cb(pose):
  rospy.loginfo(pose)
  points = point_cloud_cb(rospy.wait_for_message('/camera/depth/points', PointCloud2))
  rospy.loginfo(len(points))
  close_points = filter(lambda p: near_pose(pose, p[0], p[1], p[2]), points)
  rospy.loginfo(len(close_points))


def point_cloud_cb(point_cloud):
  # rospy.loginfo(point_cloud.point_step)
  # rospy.loginfo(point_cloud.row_step)
  # rospy.loginfo(point_cloud.fields)
  rospy.loginfo('***************')

  points = []
  points_gen = pc2.read_points(point_cloud, skip_nans=True)
  for p in points_gen:
    points.append(p)
  # rospy.loginfo(len(points))
  return points
  

def main():
  rospy.init_node('contact_detector')
  rospy.Subscriber('/contact/pose', PoseStamped, contact_cb)
  # rospy.Subscriber('/camera/depth/points', PointCloud2, point_cloud_cb)

  rospy.spin()

if __name__ == '__main__':
  main()
