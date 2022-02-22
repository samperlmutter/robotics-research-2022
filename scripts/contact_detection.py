#!/usr/bin/env python3

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
pose_proximity_radius = 0.1
cy = 239.5
cx = 319.5
fx = 570.3422
fy = 319.5


def compute_image_pos(pose):
    depth = 1
    return (((pose.x * fx) / depth) + cx,
            ((pose.y * fy) / depth) + cy)
# make bag file of all topics
# get distance from pose to camera
# convert to pixel coords
# check if nearby pixels are a similar depth
# color them if yes

def near_pose(pose_pos, point_pos):
    if ((point_pos.x - pose_pos.x) ** 2) + ((point_pos.y - pose_pos.y) ** 2) + (
            (point_pos.z - pose_pos.z) ** 2) <= pose_proximity_radius ** 2:
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

    m.pose = Pose()

    m.header.stamp = rospy.Time.now()
    m.header.frame_id = 'camera_rgb_optical_frame'

    colors = []
    for i in range(len(points)):
        colors.append(near_pose(pose.pose.position, points[i]))
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
