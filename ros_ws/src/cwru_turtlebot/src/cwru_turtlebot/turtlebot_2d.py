#!/usr/bin/env python

import random

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal

from helpers import convert_quaternion_to_yaw
from turtlebot import TurtleBot


class TurtleBot2D(TurtleBot, object):
    def __init__(self, linear_speed=.2, angular_speed=.2):
        super(TurtleBot2D, self).__init__()

        rospy.on_shutdown(self.stop)

        # initialize all variable values

        self.linear_speed = linear_speed  # expressed in m/s
        self.angular_speed = angular_speed  # expressed in rad/s
        rospy.loginfo("Completed TurtleBot2D initialization")

    def move(self, goal_x=0, goal_y=0, goal_yaw=0, x_lower_bound=-1, x_upper_bound=1, y_lower_bound=-1,
             y_upper_bound=1):
        # To prevent robot getting stuck or drifting towards just one specific area over course of experiment
        # Randomly send to middle of allowable area with 1% probability
        rand = random.uniform(0, 100)
        if rand == 1:
            goal_x = (x_lower_bound + x_upper_bound) / 2
            goal_y = (y_lower_bound + y_upper_bound) / 2

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation = self.continuous_pose_wrt_map.pose.pose.orientation  # Always keep our orientation the same

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        try:
            rospy.logdebug('Waiting for move_base server')
            self.move_base_client.wait_for_server()
            rospy.logdebug('Sending goal to move_base server')
            self.move_base_client.send_goal(goal)
            rospy.logdebug('Waiting for goal result from move_base server')
            self.move_base_client.wait_for_result()
            success = self.move_base_client.get_result()
            if not success:
                rospy.logwarn(self.namespace + ': move_base was not able to successfully complete the request action')
            else:
                continuous_x = self.continuous_pose_wrt_map.pose.pose.position.x
                continuous_y = self.continuous_pose_wrt_map.pose.pose.position.y
                discrete_x = self.discrete_pose_wrt_map.pose.pose.position.x
                discrete_y = self.discrete_pose_wrt_map.pose.pose.position.y
                gazebo_x = self.gazebo_pose_wrt_map.pose.pose.position.x
                gazebo_y = self.gazebo_pose_wrt_map.pose.pose.position.y
                rospy.loginfo(self.namespace + ': requested move to (' + str(goal_x) + ', ' + str(goal_y) + ') ' +
                              'completed. Current gazebo odom pose is (' + str(gazebo_x) + ', ' + str(
                    gazebo_y) + '). ' +
                              'Current continuous odom pose is (' + str(continuous_x) + ', ' + str(
                    continuous_y) + '). ' +
                              'Current discrete odom pose is (' + str(discrete_x) + ', ' + str(discrete_y) + '). ')
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': caught exception while sending move_base a movement goal - ' + e.message)

    def stop(self):
        try:
            self.move_base_client.cancel_all_goals()
        except rospy.ROSException as e:
            rospy.logwarn(self.namespace + ': Unable to send movement goal cancel command - ' + e.message)

    def spin(self):
        rospy.loginfo("Entering spin mode")
        cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
        command = Twist()
        command.angular.z = 0.1
        while not rospy.is_shutdown():
            cmd_vel_pub.publish(command)
            rospy.loginfo(self.namespace + ': Current yaw is ' + str(
                convert_quaternion_to_yaw(self.gazebo_pose_wrt_map.pose.pose.orientation)))
            rospy.sleep(0.1)

    @staticmethod
    def check_move_bounds(val, lower, upper):
        # check that the val is between lower and upper values
        return lower < val < upper


def main():
    # create a movable turtle bot object
    robot = TurtleBot2D()

    x_upper = rospy.get_param('/x_upper')
    x_lower = rospy.get_param('/x_lower')
    y_upper = rospy.get_param('/y_upper')
    y_lower = rospy.get_param('/y_lower')

    # move the robot back and forth randomly until process killed with ctrl-c
    while not rospy.is_shutdown():
        robot.move(goal_x=random.uniform(x_lower, x_upper),
                   goal_y=random.uniform(y_lower, y_upper),
                   x_lower_bound=x_lower,
                   x_upper_bound=x_upper,
                   y_lower_bound=y_lower,
                   y_upper_bound=y_upper)
        rospy.sleep(5)


if __name__ == '__main__':
    # run program and gracefully handle exit
    try:
        main()
    except rospy.ROSInterruptException:
        pass
