#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

import helpers

initial_position = None
gazebo_odom = None
gps_publisher = None

# TODO could integrate this into noisy_gps and use parameter to control whether or not noise is added


# Sets the initial position of the GPS
def initial_position_cb(initial_position_msg):
    global initial_position

    if initial_position is None:
        initial_position = initial_position_msg
    else:
        rospy.logwarn('Initial position callback in fake_gps triggered twice, this should not happen!')


# Save incoming odometry messages
def gazebo_odom_cb(odom_msg):
    global gazebo_odom
    gazebo_odom = odom_msg


# Publishes the GPS readings
# Must accept event argument because of use with rospy.Timer
def publish_gps(event):
    # Use this so discrete filter can localize w/o external measurements
    # FAA 2014 research shows 95% confidence interval of 3.351 meters horizontal accuracy
    # Assuming this error is Gaussian and normally distributed, we have a mean of 0 and standard deviation of 1.71
    # Obtained this by general rule of : std_dev = 95% confidence interval / 1.96
    global initial_position
    global gazebo_odom

    if None not in (initial_position, gazebo_odom):
        # Calculate robot pose wrt map
        x_map = initial_position.pose.pose.position.x + gazebo_odom.pose.pose.position.x
        y_map = initial_position.pose.pose.position.y + gazebo_odom.pose.pose.position.y

        gps = PoseWithCovarianceStamped()
        gps.header.stamp = gazebo_odom.header.stamp
        gps.header.frame_id = 'map'
        gps.pose.pose.position.x = x_map
        gps.pose.pose.position.y = y_map
        gps.pose.pose.position.z = 0
        gps.pose.pose.orientation = gazebo_odom.pose.pose.orientation
        gps.pose.covariance = gazebo_odom.pose.covariance

        assert gps_publisher is not None
        gps_publisher.publish(gps)


def main():
    debug = rospy.get_param('/debug')
    if debug:
        rospy.init_node('fake_gps', log_level=rospy.DEBUG)
    else:
        rospy.init_node('fake_gps')

    helpers.wait_for_services()

    initial_position_subscriber = rospy.Subscriber('initial_position', PoseWithCovarianceStamped, initial_position_cb)
    gazebo_odom_subscriber = rospy.Subscriber('odom_throttle', Odometry, gazebo_odom_cb)

    global gps_publisher
    gps_publisher = rospy.Publisher('fake_gps', PoseWithCovarianceStamped, queue_size=1, latch=True)

    # Publish a GPS reading once every 10 seconds
    timer = rospy.Timer(rospy.Duration(10), publish_gps)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
