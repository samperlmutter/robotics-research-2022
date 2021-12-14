#!/usr/bin/env python

import rospy
import colorsys
import math
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_from_euler

pc_pub = rospy.Publisher('/contact/cloud', Marker, queue_size=100)


def near_pose(pose_pos, point_pos):
  if ((point_pos.x - pose_pos.x) ** 2) + ((point_pos.y - pose_pos.y) ** 2) + ((point_pos.z - pose_pos.z) ** 2) <= 0.1 ** 2:
    return ColorRGBA(1, 0, 0, 1)
  else:
    return ColorRGBA(0.35, 0.35, 0.35, 1)


def contact_cb(pose):
  start = rospy.Time.now().to_time()
  points = point_cloud_cb(rospy.wait_for_message('/camera/depth/points', PointCloud2))

  m = Marker()
  m.scale = Vector3(.01, .01, .01)
  m.color = ColorRGBA(1, 1, 1, 1)
  m.type = Marker.POINTS

  q = quaternion_from_euler(math.radians(90), math.radians(180), 0)
  m.pose = Pose()
  # m.pose.orientation.x = q[0]
  # m.pose.orientation.y = q[1]
  # m.pose.orientation.z = q[2]
  # m.pose.orientation.w = q[3]

  m.header.stamp = rospy.Time.now()
  m.header.frame_id = 'camera_rgb_optical_frame'

  def calc_color(i):
    mult = 360. / float(len(points))
    return colorsys.hsv_to_rgb((i * mult) / 350., 1, 1)


  colors = [None for i in range(len(points))]
  for i in range(len(points)):
    colors[i] = near_pose(pose.pose.position, points[i])
    # c = calc_color(i)
    # colors[i] = ColorRGBA(c[0], c[1], c[2], 1)
  m.colors = colors
  m.points = points

  rospy.loginfo(rospy.Time.now().to_time() - start)
  
  pc_pub.publish(m)


def point_cloud_cb(point_cloud):
  points = []
  points_gen = pc2.read_points(point_cloud, skip_nans=True)
  for p in points_gen:
    p = Point(p[0], p[1], p[2])
    points.append(p)

  return points
  

def main():
  rospy.init_node('contact_detector')
  rospy.Subscriber('/contact/pose', PoseStamped, contact_cb)

  rospy.spin()

if __name__ == '__main__':
  main()
