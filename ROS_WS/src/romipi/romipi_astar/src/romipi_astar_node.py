#!/usr/bin/env python

# publishes:
# subscribes:
# * cmd_vel -> sends to astar

import rospy
from romipi_driver import AStar
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def cmd_vel_callback(data):
    # read cmd_vel and update the 32u4
    # with the velocity
    global romi

    linear_x = data.linear.x
    angular_z = data.angular.z
    rospy.loginfo(rospy.get_caller_id() + " forward %s, rotation %s", linear_x, angular_z)
    romi.twist(linear_x, angular_z)    

def romipi_astar_node():
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    
    rospy.init_node('romipi_astar_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "romipi_astar is alive at %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        romi = AStar()
        romipi_astar_node()
    except rospy.ROSInterruptException:
        pass

