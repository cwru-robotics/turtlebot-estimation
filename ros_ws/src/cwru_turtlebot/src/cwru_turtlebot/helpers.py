import math
import rospy
from geometry_msgs.msg import Quaternion


# Helper functions used in a variety of files across the project


# Checks if a scan has a None value in its contents, returns True if so
def scan_has_none_check(scan):
    scan_list = [scan.min, scan.max, scan.mean, scan.variance, scan.std_dev, scan.median, scan.std_error]
    # TODO could use list comprehension instead of loop
    for item in scan_list:
        if item is None:
            return True
    return False


# Converts a quaternion into its equivalent yaw value
def convert_quaternion_to_yaw(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    yaw = math.atan2(2 * (x * y + z * w), w ** 2 - z ** 2 - y ** 2 + x ** 2)
    return yaw


# Converts a yaw value into its equivalent quaternion
def convert_yaw_to_quaternion(yaw):
    # Assumes roll/pitch = 0 always since we're operating in 2D
    quaternion = Quaternion()
    quaternion.x = 0
    quaternion.y = 0
    quaternion.z = math.sin(yaw / 2)
    quaternion.w = math.cos(yaw / 2)

    return quaternion


# Correct yaw angles to a value between -pi and pi, accounting for periodicity
def correct_angle(yaw):
    while yaw > math.pi and not rospy.is_shutdown():
        yaw -= 2 * math.pi

    while yaw < -1 * math.pi and not rospy.is_shutdown():
        yaw += 2 * math.pi

    assert -1 * math.pi <= yaw <= math.pi, 'Returned yaw was %f' % yaw

    return yaw


# Function to delay startup of other nodes until Gazebo and filters have been initialized
def wait_for_services():
    # TODO add logic here to timeout and raise an error while waiting
    rospy.loginfo('Waiting for services for TurtleBot initialization...')
    # Wait for gazebo and filters to be fully initialized before starting our robot
    rospy.wait_for_service('/gazebo/set_physics_properties')
    rospy.wait_for_service('set_pose_continuous')
    rospy.wait_for_service('set_pose_discrete')
    rospy.loginfo('All required services are active')
