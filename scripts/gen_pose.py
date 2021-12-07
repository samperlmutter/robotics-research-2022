#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import random

pose_pub = rospy.Publisher('/contact/pose', PoseStamped, queue_size=10)
marker_pub = rospy.Publisher('/contact/pose/marker', Marker, queue_size=10)

def gen_pose(x, y, z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 0

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'camera_rgb_optical_frame'

    ps = PoseStamped(header, pose)

    m = Marker()
    m.scale = Vector3(2, 2, 2)
    m.color = ColorRGBA(0, 1, 0, 0.1)
    m.type = Marker.SPHERE
    header.stamp = rospy.Time.now()
    m.header = header
    m.pose = pose

    marker_pub.publish(m)

    return ps


def main():
    rospy.init_node('gen_pose', anonymous=True)

    rospy.sleep(.25)

    if len(sys.argv) >= 4:
        pose_pub.publish(gen_pose(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])))
    else:
        pose_pub.publish(gen_pose(random.uniform(0.75, 2), random.uniform(-1, 1), random.uniform(0, 0.25)))
    
if __name__ == '__main__':
    main()