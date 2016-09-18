#!/usr/bin/env python

import copy
import math
import numpy

import actionlib
import rospy

from cwru_turtlebot.msg import ExternalPoseAction, ExternalPoseGoal, ScanWithVariance, ScanWithVarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose, SetPoseRequest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt64

import helpers
from helpers import convert_quaternion_to_yaw, convert_yaw_to_quaternion


# Base class for all TurtleBots
class TurtleBot:
    def __init__(self, rate=10):
        rospy.loginfo('Starting turtlebot constructor')
        debug = rospy.get_param('/debug')
        if debug:
            rospy.init_node('robot', log_level=rospy.DEBUG)
        else:
            rospy.init_node('robot')

        # Wait for things to initialize before starting robot
        helpers.wait_for_services()

        # Create initial pose object from parameter server
        # This pose is in the map frame
        # TODO could switch this to Pose2D, for simpler usage
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.pose.pose.position.x = rospy.get_param('x_pos')
        self.initial_pose.pose.pose.position.y = rospy.get_param('y_pos')
        self.initial_pose.pose.pose.orientation = convert_yaw_to_quaternion(rospy.get_param('yaw'))
        self.initial_pose.header.frame_id = 'map'

        self.continuous_pose_wrt_map = copy.deepcopy(self.initial_pose)
        self.discrete_pose_wrt_map = copy.deepcopy(self.initial_pose)
        self.gazebo_pose_wrt_map = copy.deepcopy(self.initial_pose)

        self.initialize_publishers()

        # self.external_pose_publisher.publish()

        self.initialize_action_servers()

        self.initial_position_publisher.publish(self.initial_pose)

        self.scan_received = False  # We haven't received a valid LaserScan yet
        self.most_recent_scan = None
        self.lidar_alarm = False

        self.rate_frequency = rate
        self.rate = rospy.Rate(rate)

        self.namespace = rospy.get_namespace()[1:]  # Get rid of the leading /

        self.client_list = []  # List to hold all of our action clients
        self.existing_clients = []  # List to hold names of all action clients in the client_list
        self.external_pose_count = 0  # Number of external poses received
        self.external_pose_count_publisher.publish(UInt64(data=self.external_pose_count))
        self.external_pose_publisher.publish(self.initial_pose)  # Publish so that we start out knowing where we are
        self.initialize_action_clients()

        self.initialize_subscribers()

        # Wait for everything else in Gazebo world to be ready
        # rospy.sleep(2)  # Sleep to avoid crash due to get/set of param in rapid succession,
        self.wait_for_clients()

        # Once everything is ready we need to reset our filters
        # because they could have gotten erroneous readings
        self.reset_filters()
        rospy.loginfo('Finished base TurtleBot initialization')

    def initialize_subscribers(self):
        self.lidar_subscriber = rospy.Subscriber('scan',
                                                 LaserScan,
                                                 self.scan_callback)

        self.continuous_odom_subscriber = rospy.Subscriber('odometry/filtered_continuous',
                                                           Odometry,
                                                           self.continuous_odom_callback)

        self.discrete_odom_subscriber = rospy.Subscriber('odometry/filtered_discrete',
                                                         Odometry,
                                                         self.discrete_odom_callback)

        self.gazebo_odom_subscriber = rospy.Subscriber('odom_throttle',
                                                       Odometry,
                                                       self.gazebo_odom_callback)

    def initialize_publishers(self):
        # TODO do we really need this anymore?
        self.processed_scan_publisher = rospy.Publisher('processed_scan',
                                                        ScanWithVarianceStamped,
                                                        queue_size=1)

        self.initial_position_publisher = rospy.Publisher('initial_position',
                                                          PoseWithCovarianceStamped,
                                                          queue_size=1,
                                                          latch=True)

        # Publisher for poses received from other robots, used as input to UKF
        self.external_pose_publisher = rospy.Publisher('external_poses',
                                                       PoseWithCovarianceStamped,
                                                       queue_size=10)

        self.external_pose_count_publisher = rospy.Publisher('external_poses_count',
                                                             UInt64,
                                                             queue_size=1,
                                                             latch=True)

    def initialize_action_servers(self):
        self.external_pose_as = actionlib.SimpleActionServer('external_pose_action',
                                                             ExternalPoseAction,
                                                             execute_cb=self.external_pose_cb,
                                                             auto_start=False)
        self.external_pose_as.start()
        rospy.set_param('server_started', True)

    def initialize_action_clients(self):
        self.update_client_list()

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def update_client_list(self):
        rospy.logdebug(self.namespace + ': Updating client list...')
        param_list = rospy.get_param_names()  # Get list of strings with all parameter names on the parameter server
        for param in param_list:
            if (param.endswith("server_started") and
                    not param.startswith(rospy.get_namespace()) and
                        rospy.get_param(param) is True):
                # Parameter indicates action server exists, is a different robot, and has been started
                server_name = param.replace("server_started", "external_pose_action")
                rospy.logdebug(self.namespace + ': Found server with name: ' + server_name)
                if server_name not in self.existing_clients:
                    # This server has not previously been added so let's make a new client for it
                    new_client = actionlib.SimpleActionClient(server_name, ExternalPoseAction)
                    self.client_list.append(new_client)
                    self.existing_clients.append(server_name)
                    rospy.logdebug(self.namespace + ': Added server with name ' + server_name + ' to client list.')

        rospy.logdebug("Updated client list, it now contains %d robots", len(self.existing_clients))

    def initialize_scanner(self, scan_msg):
        # Scan properties
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment
        self.time_increment = scan_msg.time_increment
        self.scan_time = scan_msg.scan_time
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max

        # Don't reinitialize scanner multiple times
        self.scan_received = True

    def scan_callback(self, scan_msg):
        # rospy.logdebug(self.namespace + ': Entering scan callback')
        # initialize scanner properties for this robot
        if not self.scan_received:
            self.initialize_scanner(scan_msg)

        # First throw out invalid particles
        valid_particles = []
        min_angle = self.angle_max
        max_angle = self.angle_min
        # ranges = numpy.array(scan_msg.ranges)
        for index, particle in enumerate(scan_msg.ranges):
            if self.range_min <= particle <= self.range_max:
                valid_particles.append(particle)
                angle = self.angle_min + index * self.angle_increment  # Calculate the yaw offset of this particle
                if angle > max_angle:
                    max_angle = angle
                if angle < min_angle:
                    min_angle = angle

        scan = ScanWithVariance()

        # Find the min, max, mean, median, variance, std dev, and std error if there are particles
        if len(valid_particles) is not 0:
            scan.min = numpy.min(valid_particles)
            scan.max = numpy.max(valid_particles)
            scan.mean = numpy.mean(valid_particles)
            scan.median = numpy.median(valid_particles)
            scan.variance = numpy.var(valid_particles)
            scan.std_dev = math.sqrt(scan.variance)  # standard deviation is square root of variance
            scan.std_error = scan.std_dev / math.sqrt(len(valid_particles))
            scan.yaw_offset = (max_angle + min_angle) / 2  # average yaw of the valid particles
            scan.valid = True
            # rospy.logdebug(self.namespace + ': Received valid scan with median distance ' + str(scan.median))
        else:
            # otherwise scan is not valid
            scan.valid = False
            # rospy.logdebug(self.namespace + ': Received invalid scan')

        if scan.valid:
            # Check whether we need to activate lidar alarm
            if 0 < scan.median < 0.5:
                self.lidar_alarm = True
            else:
                self.lidar_alarm = False

            processed_scan = self.stamp_scan_w_variance(scan)

            try:
                self.processed_scan_publisher.publish(processed_scan)
            except rospy.ROSException as e:
                rospy.logwarn(self.namespace + ': Unable to publish most recent processed scan - ' + e.message)

            self.most_recent_scan = processed_scan
            self.send_scan_to_clients(processed_scan)
            # rospy.logdebug(self.namespace + ': Exiting scan callback')

    def continuous_odom_callback(self, odom):
        # Must add initial pose value to convert from odom frame to map frame
        # -20 is because our map is currently 40x40 with o
        self.continuous_pose_wrt_map.pose.pose.position.x = odom.pose.pose.position.x + self.initial_pose.pose.pose.position.x
        self.continuous_pose_wrt_map.pose.pose.position.y = odom.pose.pose.position.y + self.initial_pose.pose.pose.position.y

        odom_yaw = convert_quaternion_to_yaw(odom.pose.pose.orientation)
        initial_yaw = convert_quaternion_to_yaw(self.initial_pose.pose.pose.orientation)
        current_yaw = odom_yaw + initial_yaw
        self.continuous_pose_wrt_map.pose.pose.orientation = convert_yaw_to_quaternion(current_yaw)
        self.continuous_pose_wrt_map.header.stamp = odom.header.stamp

    def discrete_odom_callback(self, odom):
        # Already in map frame so no need to convert
        self.discrete_pose_wrt_map.pose.pose.position.x = odom.pose.pose.position.x
        self.discrete_pose_wrt_map.pose.pose.position.y = odom.pose.pose.position.y
        self.discrete_pose_wrt_map.pose.pose.orientation = odom.pose.pose.orientation
        self.discrete_pose_wrt_map.header.stamp = odom.header.stamp

    def gazebo_odom_callback(self, odom):
        # Must add initial pose value to convert from odom frame to map frame
        self.gazebo_pose_wrt_map.pose.pose.position.x = odom.pose.pose.position.x + self.initial_pose.pose.pose.position.x
        self.gazebo_pose_wrt_map.pose.pose.position.y = odom.pose.pose.position.y + self.initial_pose.pose.pose.position.y

        odom_yaw = convert_quaternion_to_yaw(odom.pose.pose.orientation)
        initial_yaw = convert_quaternion_to_yaw(self.initial_pose.pose.pose.orientation)
        current_yaw = odom_yaw + initial_yaw
        self.gazebo_pose_wrt_map.pose.pose.orientation = convert_yaw_to_quaternion(current_yaw)
        self.gazebo_pose_wrt_map.header.stamp = odom.header.stamp

    def send_scan_to_clients(self, scan):
        # rospy.logdebug(self.namespace + ': Sending scan to clients')
        pose = self.convert_scan_to_pose(scan)
        self.update_client_list()

        rospy.logdebug(self.namespace + ': There are ' + str(len(self.client_list)) + ' clients in the list')
        for client in self.client_list:
            try:
                rospy.logdebug(self.namespace + ': Waiting for client server')
                client.wait_for_server()
                goal = ExternalPoseGoal(pose)
                client.send_goal(goal)
                client.wait_for_result()
                success = client.get_result()
                if not success:
                    rospy.logwarn(self.namespace + ': A client returned unsucessfully when sent a pose measurement')
            except rospy.ROSException as e:
                rospy.logwarn(self.namespace + ': Sending scan to clients caught exception: ' + e.message)
                # rospy.logdebug(self.namespace + ': Finished sending scan to clients')

    def convert_scan_to_pose(self, scan):
        # rospy.logdebug(self.namespace + ': Converting scan to pose')
        pose = None
        if scan.scan.valid is True:
            # Scan is valid so we can create a pose from it
            pose = PoseWithCovarianceStamped()

            # Determine our current pose (which is already in the map frame)
            current_x = self.discrete_pose_wrt_map.pose.pose.position.x
            current_y = self.discrete_pose_wrt_map.pose.pose.position.y
            # TODO find why discrete filter is not correctly getting orientation
            current_yaw = convert_quaternion_to_yaw(
                self.gazebo_pose_wrt_map.pose.pose.orientation) + scan.scan.yaw_offset

            # Determine what pose (in the map frame) we believe we are seeing the other robot at
            # Add 0.2 (turtlebot radius) to more evenly distribute pose estimates non-deterministically
            pose.pose.pose.position.x = current_x + scan.scan.median * math.cos(current_yaw)
            pose.pose.pose.position.y = current_y + scan.scan.median * math.sin(current_yaw)

            continuous_x = self.continuous_pose_wrt_map.pose.pose.position.x
            continuous_y = self.continuous_pose_wrt_map.pose.pose.position.y
            continuous_yaw = convert_quaternion_to_yaw(self.continuous_pose_wrt_map.pose.pose.orientation)
            discrete_x = self.discrete_pose_wrt_map.pose.pose.position.x
            discrete_y = self.discrete_pose_wrt_map.pose.pose.position.y
            discrete_yaw = convert_quaternion_to_yaw(self.discrete_pose_wrt_map.pose.pose.orientation)
            gazebo_x = self.gazebo_pose_wrt_map.pose.pose.position.x
            gazebo_y = self.gazebo_pose_wrt_map.pose.pose.position.y
            gazebo_yaw = convert_quaternion_to_yaw(self.gazebo_pose_wrt_map.pose.pose.orientation)

            rospy.logdebug(self.namespace + ': Current gazebo odom pose is (' + str(gazebo_x) + ', ' + str(gazebo_y)
                           + ', ' + str(gazebo_yaw) + '). '
                           + 'Current continuous odom pose is (' + str(continuous_x) + ', ' + str(continuous_y) + ', '
                           + str(continuous_yaw) + '). '
                           + 'Current discrete odom pose is (' + str(discrete_x) + ', ' + str(discrete_y) + ', '
                           + str(discrete_yaw) + '). '
                           + ' got scan with median ' + str(scan.scan.median) +
                           ' and calculated pose of other robot at (' + str(pose.pose.pose.position.x) + ', ' +
                           str(pose.pose.pose.position.y) + ').')
            # Right now use default quaternion since we can't figure out orientation from our scans
            pose.pose.pose.orientation.x = 0
            pose.pose.pose.orientation.y = 0
            pose.pose.pose.orientation.z = 0
            pose.pose.pose.orientation.w = 1
            pose.header.frame_id = 'map'
            pose.header.stamp = scan.header.stamp

            # TODO figure out covariances for these poses
            pose.pose.covariance[0] = scan.scan.variance
            pose.pose.covariance[7] = scan.scan.variance
        else:
            rospy.logdebug(self.namespace + ': Scan received for conversion to pose was not valid')

        # rospy.logdebug(self.namespace + ': Finished converting scan to pose')
        return pose

    def external_pose_cb(self, goal):
        # rospy.logdebug(self.namespace + ': Entering external pose callback')

        success = False
        try:
            continuous_x = self.continuous_pose_wrt_map.pose.pose.position.x
            continuous_y = self.continuous_pose_wrt_map.pose.pose.position.y
            continuous_yaw = convert_quaternion_to_yaw(self.continuous_pose_wrt_map.pose.pose.orientation)
            discrete_x = self.discrete_pose_wrt_map.pose.pose.position.x
            discrete_y = self.discrete_pose_wrt_map.pose.pose.position.y
            discrete_yaw = convert_quaternion_to_yaw(self.discrete_pose_wrt_map.pose.pose.orientation)
            gazebo_x = self.gazebo_pose_wrt_map.pose.pose.position.x
            gazebo_y = self.gazebo_pose_wrt_map.pose.pose.position.y
            gazebo_yaw = convert_quaternion_to_yaw(self.gazebo_pose_wrt_map.pose.pose.orientation)
            rospy.logdebug(self.namespace + ': Received external pose indicating position (' +
                           str(goal.pose.pose.pose.position.x) + ', ' + str(goal.pose.pose.pose.position.y) + '). ' +
                           'Current gazebo odom pose is (' + str(gazebo_x) + ', ' + str(gazebo_y)
                           + ', ' + str(gazebo_yaw) + '). '
                           + 'Current continuous odom pose is (' + str(continuous_x) + ', ' + str(continuous_y) + ', '
                           + str(continuous_yaw) + '). '
                           + 'Current discrete odom pose is (' + str(discrete_x) + ', ' + str(discrete_y) + ', '
                           + str(discrete_yaw) + '). ')
            self.external_pose_publisher.publish(goal.pose)
            self.external_pose_count += 1
            self.external_pose_count_publisher.publish(UInt64(data=self.external_pose_count))
            success = True
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': Unable to publish most recent external pose - ' + e.message)

        if success:
            self.external_pose_as.set_succeeded()
        rospy.logdebug(self.namespace + ': Exiting external pose callback')

    def wait_for_clients(self):
        self.update_client_list()
        num_robots = rospy.get_param('/number_of_robots')
        complete = False
        while not rospy.is_shutdown() and not complete:
            if num_robots - 1 != len(self.existing_clients):
                rospy.logdebug(self.namespace + ': Waiting for other robots to come online...' +
                               'expecting ' + str(num_robots - 1) + ' other robots in world, and currently have ' +
                               str(len(self.existing_clients)) + ' in client list.')
                self.rate.sleep()
                self.update_client_list()
            else:
                complete = True

    @staticmethod
    def stamp_scan_w_variance(scan_w_variance):
        # Create time-stamped scan message including the scan and variance of points
        stamped_scan_w_variance = ScanWithVarianceStamped()
        stamped_scan_w_variance.scan = scan_w_variance
        stamped_scan_w_variance.header.stamp = rospy.get_rostime()

        return stamped_scan_w_variance

    def reset_filters(self):
        rospy.loginfo(self.namespace + ': Resetting filters...')
        error = False
        # TODO add logic here to timeout while waiting and raise an error
        # First reset the continuous filter
        rospy.wait_for_service('set_pose_continuous')
        try:
            continuous_service = rospy.ServiceProxy('set_pose_continuous', SetPose)
            request = SetPoseRequest()
            continuous_service(request)
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': Service call failed - ' + e.message)
            error = True

        # Next reset the discrete filter
        rospy.wait_for_service('set_pose_discrete')
        try:
            continuous_service = rospy.ServiceProxy('set_pose_discrete', SetPose)
            request = SetPoseRequest()
            continuous_service(request)
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': Service call failed - ' + e.message)
            error = True

        if not error:
            rospy.logdebug(self.namespace + ': Filter reset complete')
        else:
            rospy.logwarn(self.namespace + ': Filter reset encountered errors')
