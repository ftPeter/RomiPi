#!/usr/bin/env python

# publishes:
# subscribes:
# * cmd_vel -> sends to astar

# See turtlebot3 example from here:
# https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_burger/turtlebot3_core/turtlebot3_core.ino

import rospy

import sys
for p in sys.path:
    print p

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

class RomiPi():
    def __init__(self):
        self.romi = AStar()

        rospy.init_node('romipi_astar_node', anonymous=True)

        self.battery_publisher = rospy.Publisher('battery_state', BatteryState, queue_size = 2)
        self.odom_state_publisher = rospy.Publisher('odom', Odometry, queue_size = 2)
        self.joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size = 2)
        self.tf_broadcaster = TransformBroadcaster()


    def cmd_vel_callback(self, data):
        # read cmd_vel and update the 32u4
        # with the velocity
        linear_x = data.linear.x
        angular_z = data.angular.z
        #rospy.loginfo(rospy.get_caller_id() + " forward %s, rotation %s", linear_x, angular_z)
        self.romi.twist(linear_x, angular_z)    

    def broadcast_tf_msg(self, broadcaster):
        odom_tf = TransformStamped()
       
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_footprint"

        # FIXME confirm units on read_pose_coordinate to be in meters
        odom_tf.transform.translation.x = self.odom_pose[0]
        odom_tf.transform.translation.y = self.odom_pose[1]
        odom_tf.transform.translation.z = 0.0
        
        quaternion = quaternion_from_euler(0, 0, self.odom_pose[2])
        #type(pose) = geometry_msgs.msg.Pose
        odom_tf.transform.rotation.x = quaternion[0]
        odom_tf.transform.rotation.y = quaternion[1]
        odom_tf.transform.rotation.z = quaternion[2]
        odom_tf.transform.rotation.w = quaternion[3]

        odom_tf.header.stamp = rospy.get_rostime()
        broadcaster.sendTransformMessage(odom_tf)

    def publish_odom_state_msg(self, publisher):
        odom_state_msg = Odometry()

        # FIXME check that this shouldn't be /odom
        odom_state_msg.header.frame_id = "odom"
        # FIXME check that this shouldn't be /base_footprint
        odom_state_msg.child_frame_id = "base_footprint"

        # FIXME confirm units on read_pose_coordinate to be in meters
        odom_state_msg.pose.pose.position.x = self.odom_pose[0]
        odom_state_msg.pose.pose.position.y = self.odom_pose[1]
        odom_state_msg.pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0, 0, self.odom_pose[2])
        #type(pose) = geometry_msgs.msg.Pose
        odom_state_msg.pose.pose.orientation.x = quaternion[0]
        odom_state_msg.pose.pose.orientation.y = quaternion[1]
        odom_state_msg.pose.pose.orientation.z = quaternion[2]
        odom_state_msg.pose.pose.orientation.w = quaternion[3]
        # FIXME confirm units on read_pose_twist to be m/s and rad/s
        odom_state_msg.twist.twist.linear.x = self.odom_vel[0]
        odom_state_msg.twist.twist.angular.z = self.odom_vel[1]
        
        odom_state_msg.header.stamp = rospy.get_rostime()
        publisher.publish(odom_state_msg)

    def publish_joint_state_msg(self, publisher):
        joint_state_msg = JointState()
        joint_state_msg.name = ["wheel_left_joint", "wheel_right_joint"]

        # FIXME wheel position is just the encoder value, convert to radians
        joint_state_msg.position = [self.left_wheel_position, self.right_wheel_position]
        # FIXME confirm romi returns velocity in rad/s 
        joint_state_msg.velocity = [self.left_wheel_velocity, self.right_wheel_velocity]
        # we have no effort sensing, so leave blank
        joint_state_msg.effort = []
        
        joint_state_msg.header.stamp = rospy.get_rostime() 
        publisher.publish(joint_state_msg)

    def publish_battery_state_msg(self, publisher):

        battery_state_msg = BatteryState()
        battery_state_msg.header.stamp = rospy.get_rostime()
        battery_state_msg.design_capacity = 1.9
        battery_state_msg.voltage = self.battery_mv / 1000.0
        # 9.0 is 1.5 volts (for full nimh AA) times 6 batteries in series
        # 7.2 is 1.2 volts (for working nimh) times 6 batteries
        # 6.0 is the cut-off (empty)
        # so percentage = 1/3 * battery_v - 2
        # this method is not super accurate but better than nothing
        # as NimH battery discharge is very non-linear.
        # note percentage is 0-1.0 and not really a percentage per se.
        battery_state_msg.percentage = (battery_state_msg.voltage / 3.0) - 2.0
        if( self.battery_mv >= 0.00 ):
            battery_state_msg.present = True
        else:
            battery_state_msg.present = False

        publisher.publish(battery_state_msg)

    def read_romi_state(self):
        # add error handling here.
        # try-catch timeout type deal maybe?
        try:
            # read all of the outputs
            self.battery_mv = self.romi.read_battery_millivolts()
            self.left_wheel_position, self.right_wheel_position = self.romi.read_encoders()
            self.left_wheel_velocity, self.right_wheel_velocity = self.romi.read_pose_motors()
            self.odom_pose = self.romi.read_pose_coordinate()
            self.odom_vel = self.romi.read_pose_twist() 
            self.odom_pose = self.romi.read_pose_coordinate()
        except:
            rospy.logfatal("reading form Romi I2C driver failed.")
        
    def romipi_astar_node(self):
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.read_romi_state()

            self.publish_battery_state_msg(self.battery_publisher)
            self.publish_odom_state_msg(self.odom_state_publisher)
            self.publish_joint_state_msg(self.joint_state_publisher)
            self.broadcast_tf_msg(self.tf_broadcaster)

            rate.sleep()

if __name__ == '__main__':
    try:
        romi = RomiPi()
        romi.romipi_astar_node()
    except rospy.ROSInterruptException:
        pass

