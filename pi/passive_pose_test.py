#!/usr/bin/python2
#
# Print out the pose estimates
# without setting twist so that the
# user can push the robot around

from romipi_driver import AStar
import time


romi = AStar()
romi.reset_encoders()
print("Firmware Version: ", romi.read_firmware_version())
print("Battery:          ", romi.read_battery_millivolts(), " mV")
print("Encoders (l,r):  ", romi.read_encoders() )

while True:
        print "Encoders (l,r):  ", romi.read_encoders()
        print "Motor Targets (l,r): ", romi.read_pose_motors()
        print "Twist (linear m/s, rotation rad/s): %0.2f, %0.2f" % romi.read_pose_twist()
        print "Pose (x m,y m): %0.2f, %0.2f" % romi.read_pose_coordinate()
        print "Quaternion (z,w): %0.2f, %0.2f" % romi.read_quat()
        time.sleep(0.5)

