#!/usr/bin/env python
import rospy


# Use this node (with required attribute set to true) to control the simulation runtime
def main():
    debug = rospy.get_param('/debug')
    if debug:
        rospy.init_node('simulation_timer', log_level=rospy.DEBUG)
    else:
        rospy.init_node('simulation_timer')

    # Don't use helpers.wait_for_services() because it relies on namespace completion for the set_pose_* service calls
    rospy.wait_for_service('/gazebo/set_physics_properties')

    sim_time = rospy.get_param('sim_time')  # This is specified in seconds
    interval = float(sim_time) / 10
    timer = rospy.Duration.from_sec(interval)
    for i in range(0, 10):
        count = 10 * i
        rospy.loginfo("Simulation is " + str(count) + "% complete...")
        rospy.sleep(timer)


if __name__ == '__main__':
    # run program and gracefully handle exit
    try:
        main()
    except rospy.ROSInterruptException:
        pass
