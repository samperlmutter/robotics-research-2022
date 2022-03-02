from geometry_msgs.msg import Point, Pose

cy = 239.5
cx = 319.5
fx = 570.3422
fy = 319.5


# world coord to pixel
def pose_to_pixel(pose):
    depth = pose.position.z
    point = Point()

    point.x = ((pose.position.x * fx) / depth) + cx
    point.y = ((pose.position.y * fy) / depth) + cy
    point.z = depth

    return point


# pixel to world coord
def pixel_to_pose(x, y, depth):
    pose = Pose()
    pose.position.x = (x - cx) * depth / fx
    pose.position.y = (y - cy) * depth / fy
    pose.position.z = depth

    return pose
