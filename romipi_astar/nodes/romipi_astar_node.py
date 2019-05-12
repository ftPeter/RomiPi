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
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler

LEFT = 0
RIGHT = 1

def cmd_vel_callback(data):
    # read cmd_vel and update the 32u4
    # with the velocity
    global romi

    linear_x = data.linear.x
    angular_z = data.angular.z
    rospy.loginfo(rospy.get_caller_id() + " forward %s, rotation %s", linear_x, angular_z)
    romi.twist(linear_x, angular_z)    

def broadcast_tf_msg(broadcaster):
    global romi

    odom_tf = TransformStamped()
   
    # TODO just use the equivalent data from odom_state_msg
    # instead of having different data in tf
    odom_tf.header.frame_id = "odom"
    odom_tf.child_frame_id = "base_footprint"

    # FIXME confirm units on read_pose_coordinate to be in meters
    odom_pose = romi.read_pose_coordinate()
    odom_tf.transform.translation.x = odom_pose[0]
    odom_tf.transform.translation.y = odom_pose[1]
    odom_tf.transform.translation.z = 0.0
    
    quaternion = quaternion_from_euler(0, 0, odom_pose[2])
    #type(pose) = geometry_msgs.msg.Pose
    odom_tf.transform.rotation.x = quaternion[0]
    odom_tf.transform.rotation.y = quaternion[1]
    odom_tf.transform.rotation.z = quaternion[2]
    odom_tf.transform.rotation.w = quaternion[3]

    odom_tf.header.stamp = rospy.get_rostime()
    broadcaster.sendTransformMessage(odom_tf)

def publish_odom_state_msg(publisher):
    global romi

    odom_state_msg = Odometry()

    # FIXME check that this shouldn't be /odom
    odom_state_msg.header.frame_id = "odom"
    # FIXME check that this shouldn't be /base_footprint
    odom_state_msg.child_frame_id = "base_footprint"

    # FIXME confirm units on read_pose_coordinate to be in meters
    odom_pose = romi.read_pose_coordinate()
    odom_state_msg.pose.pose.position.x = odom_pose[0]
    odom_state_msg.pose.pose.position.y = odom_pose[1]
    odom_state_msg.pose.pose.position.z = 0.0

    quaternion = quaternion_from_euler(0, 0, odom_pose[2])
    #type(pose) = geometry_msgs.msg.Pose
    odom_state_msg.pose.pose.orientation.x = quaternion[0]
    odom_state_msg.pose.pose.orientation.y = quaternion[1]
    odom_state_msg.pose.pose.orientation.z = quaternion[2]
    odom_state_msg.pose.pose.orientation.w = quaternion[3]

    # FIXME confirm units on read_pose_twist to be m/s and rad/s
    odom_vel = romi.read_pose_twist() 
    odom_state_msg.twist.twist.linear.x = odom_vel[0]
    odom_state_msg.twist.twist.angular.z = odom_vel[1]

    
    odom_state_msg.header.stamp = rospy.get_rostime()
    publisher.publish(odom_state_msg)

def publish_joint_state_msg(publisher):
    global romi

    joint_state_msg = JointState()
    joint_state_msg.name = ["wheel_left_joint", "wheel_right_joint"]

    # FIXME wheel position is just the encoder value, convert to radians
    # left_wheel_position, right_wheel_position = romi.read_encoders()
    left_wheel_position = 0.0
    right_wheel_position = 0.0
    joint_state_msg.position = [left_wheel_position, right_wheel_position]

    # FIXME confirm romi returns velocity in rad/s 
    #left_wheel_velocity, right_wheel_velocity = romi.read_pose_motors()
    left_wheel_velocity = 0.0
    right_wheel_velocity = 0.0
    joint_state_msg.velocity = [left_wheel_velocity, right_wheel_velocity]
    joint_state_msg.effort = []
    
    joint_state_msg.header.stamp = rospy.get_rostime() 
    publisher.publish(joint_state_msg)

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
    odom_state_publisher = rospy.Publisher('odom', Odometry, queue_size = 2)
    joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size = 2)
    tf_broadcaster = TransformBroadcaster()

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "romipi_astar is alive at %s" % rospy.get_time()

        publish_battery_state_msg(battery_publisher)
        publish_odom_state_msg(odom_state_publisher)
        publish_joint_state_msg(joint_state_publisher)
        broadcast_tf_msg(tf_broadcaster)
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        romi = AStar()
        romipi_astar_node()
    except rospy.ROSInterruptException:
        pass

