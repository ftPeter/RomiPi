#!/usr/bin/python2

import rospy
from std_msgs.msg import Int16
from a_star2 import AStar as Romi

def romi_status():
    romi = Romi()
    pub = rospy.Publisher('battery_voltage', Int16, queue_size=10)
    rospy.init_node('romi_32u4_status')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        voltage = read_battery_millivolts()
        pub.publish(Int16(voltage))
        rate.sleep()
  
if __name__ == '__main__':
    try:
        romi_status()
    except rospy.ROSInterruptException:
        pass
