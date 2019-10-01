#!/usr/bin/env python3
#
# TODO 
# * Fix so that rotation numbers work out better. Try mapping the velocity to
#   the linear and angular velocities.
# * Do I need the Willow Garage License? This barely uses the example code anymore
# 
# Running This:
# * roscore
# * rosrun romipi_i2c romi_i2c.py
# * rostopic echo left_wheel_encoder
# * roslaunch romipi_teleop romipi_teleop_key.launch
#
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from a_star import AStar
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist, Quaternion, Pose, Point
from nav_msgs.msg import Odometry

left_motor = 0
right_motor = 0

def cmd_vel_callback(data):
    global left_motor
    global right_motor

    linear_x = data.linear.x
    angular_z = data.angular.z
    #rospy.loginfo(rospy.get_caller_id() + " forward %s, rotation %s", linear_x, angular_z)
    
    left_motor, right_motor = twist_to_motor(linear_x, angular_z)


def twist_to_motor(linear_vel, angle_vel):
    # convert to motor speeds in m/s
    wheel_dist_m = 0.14
    right_sp = (angle_vel * wheel_dist_m) / 2 + linear_vel
    left_sp = linear_vel * 2 - right_sp

    return (left_sp, right_sp)

def romipi_i2c():
    sequence = 0
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

    left_wheel_pub = rospy.Publisher('left_wheel_encoder', Int32, queue_size=10)
    right_wheel_pub = rospy.Publisher('right_wheel_encoder', Int32, queue_size=10)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

    rospy.init_node('romi_i2c', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    romi.reset_encoders()
    left_enc, right_enc = romi.read_encoders()
    
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        left_enc, right_enc = romi.read_encoders()

        left_wheel_pub.publish(left_enc)
        right_wheel_pub.publish(right_enc)

        #rospy.loginfo("left_motor %0.2f, right_motor %0.2f", left_motor, right_motor)
        romi.motor_velocities(left_motor, right_motor)

        x,y,qz,qw,tx,tz = romi.read_pose()
        #rospy.loginfo("Raw Pose <= pos %0.3f,%0.3f,%0.3f, quat %0.3f,%0.3f,%0.3f,%0.3f twist %0.4f,%0.4f", x,y,0, 0,0,qz,qw, tx,tz)
        romi_odom = Odometry()
        romi_odom.header.frame_id = "odom"
        romi_odom.header.stamp = rospy.Time.now()
        
        romi_odom.header.seq = sequence
        sequence += 1
        romi_odom.child_frame_id = "base_link"
        romi_odom.pose.pose.position    = Point(x=x, y=y, z=0.0)
        romi_odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        romi_odom.twist.twist.linear.x = tx
        romi_odom.twist.twist.linear.y = 0.0
        romi_odom.twist.twist.linear.z = 0.0
        romi_odom.twist.twist.angular.x = 0.0
        romi_odom.twist.twist.angular.y = 0.0
        romi_odom.twist.twist.angular.z = tz

        odom_pub.publish(romi_odom)
        rospy.loginfo("Odometry %s", romi_odom)

        rate.sleep()

if __name__ == '__main__':
    try:
        romi = AStar()
        romipi_i2c()
    except rospy.ROSInterruptException:
        pass
    romi.close() 
