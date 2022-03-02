from geometry_msgs.msg import Pose
from ee_pose_detection.msg import DepthPixel

cy = 239.5
cx = 319.5
fx = 570.3422
fy = 319.5


# world coord to pixel
def pose_to_pixel(pose):
    depth = pose.position.z
    pixel = DepthPixel()

    pixel.x = int(((pose.position.x * fx) / depth) + cx)
    pixel.y = int(((pose.position.y * fy) / depth) + cy)
    pixel.depth = depth

    return pixel


# pixel to world coord
def pixel_to_pose(x, y, depth):
    pose = Pose()
    pose.position.x = (x - cx) * depth / fx
    pose.position.y = (y - cy) * depth / fy
    pose.position.z = depth

    return pose
