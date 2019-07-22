#!/usr/bin/python2
#
# Move the robot forwards and backwards
from romipi_astar.romipi_driver import AStar
import time

motor_delay_s = 2.0
forward_speed_m_s = 0.1
stop_speed_m_s = 0.0
romi = AStar()

def monitor_pose(duration_s):
    monitor_freq_hz = 2.0

    # move forward for two seconds
    for i in range(0, int(duration_s*monitor_freq_hz)):
        print("{:} twist {:}"
              " motor speed m/s {:}"
              " pose_twist {:}".format(
            i / monitor_freq_hz,
            romi.read_twist(),
            romi.read_pose_motors(),
            romi.read_pose_twist()))
        # print motor_speeds for two seconds
        time.sleep(1.0/monitor_freq_hz)

print("initializing...")
romi.pixels(0,0,255)

print("moving forwards...")
# twist format is forward vector, rotation vector
romi.twist(forward_speed_m_s, stop_speed_m_s)
romi.pixels(0,255,0)

# move forward for two seconds
monitor_pose(2.0)

print("stopping.")
romi.twist(stop_speed_m_s, stop_speed_m_s)
romi.pixels(0,0,255)

monitor_pose(10.0)
