#!/usr/bin/env python

# publishes:
# subscribes:
# * cmd_vel -> sends to astar

# See turtlebot3 example from here:
# https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_burger/turtlebot3_core/turtlebot3_core.ino

import rospy
from romipi_astar.romipi_driver import AStar

# message types
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

def cmd_vel_callback(data):
    # read cmd_vel and update the 32u4
    # with the velocity
    global romi

    linear_x = data.linear.x
    angular_z = data.angular.z
    rospy.loginfo(rospy.get_caller_id() + " forward %s, rotation %s", linear_x, angular_z)
    romi.twist(linear_x, angular_z)    

def update_romi_state():
    global romi

    encoders = romi.read_encoders()
    pose = romi.read_pose_coordinate()
    quaternion = romi.read_quat()
    motor_vel = romi.read_pose_motors()
    pose_twist  = romi.read_pose_twist()

def publish_battery_state_msg(publisher):
    battery_mv = romi.read_battery_millivolts()

    battery_state_msg = BatteryState()
    battery_state_msg.header.stamp = rospy.get_rostime()
    battery_state_msg.design_capacity = 1.9
    battery_state_msg.voltage = battery_mv / 1000.0
    # 9.0 is 1.5 volts (for full nimh AA) times 6 batteries in series
    # 7.2 is 1.2 volts (for working nimh) times 6 batteries
    # 6.0 is the cut-off (empty)
    # so percentage = 1/3 * battery_v - 2
    # this method is not super accurate but better than nothing
    # as NimH battery discharge is very non-linear.
    # note percentage is 0-1.0 and not really a percentage per se.
    battery_state_msg.percentage = (battery_state_msg.voltage / 3.0) - 2.0
    if( battery_mv >= 0.00 ):
        battery_state_msg.present = True
    else:
        battery_state_msg.present = False

    publisher.publish(battery_state_msg)

def romipi_astar_node():
    rospy.init_node('romipi_astar_node', anonymous=True)

    battery_publisher = rospy.Publisher('battery_state', BatteryState, queue_size = 2)

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "romipi_astar is alive at %s" % rospy.get_time()

        publish_battery_state_msg(battery_publisher)

        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        romi = AStar()
        romipi_astar_node()
    except rospy.ROSInterruptException:
        pass

