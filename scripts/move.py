#!/usr/bin/env python3

import sys
import rospy
from moveit_msgs.msg import RobotState
import moveit_commander
from geometry_msgs.msg import Pose
from fetch_msgs.srv import MoverService, MoverServiceResponse
from sensor_msgs.msg import JointState


joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]


def to_RUF(pose):
    pose.position.x = -pose.position.y
    pose.position.y = pose.position.z
    pose.position.z = pose.position.x
    pose.orientation.x = -pose.orientation.y
    pose.orientation.y = pose.orientation.z
    pose.orientation.z = pose.orientation.x

    return pose


def to_FLU(pose):
    pose.position.x = pose.position.z
    pose.position.y = -pose.position.x
    pose.position.z = pose.position.y
    pose.orientation.x = pose.orientation.z
    pose.orientation.y = -pose.orientation.x
    pose.orientation.z = pose.orientation.y

    return pose


def move(req):
    group_name = 'arm'  # also 'arm_with_torso' and 'gripper'
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = req.joints_input.joints

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(req.pose.pose)
    plan = move_group.plan()

    if not plan:
        rospy.logerr(f'Trajectory could not be planned for a destination of {req.pose.pose} with starting joint angles {req.joints_input.joints}')

    res = MoverServiceResponse()
    res.trajectories.append(plan[1])

    move_group.clear_pose_targets()

    return res


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('fetch_moveit', anonymous=True)

    s = rospy.Service('fetch_moveit', MoverService, move)
    rospy.spin()
