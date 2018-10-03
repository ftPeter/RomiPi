#!/usr/bin/python3
#
# Move the robot forwards and backwards
from a_star import AStar
import time

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('twist_linear', type=float, 
                    help='linear component of twist in m/s')
parser.add_argument('twist_angular', type=float, 
                    help='angular component of twist in deg/s')
args = parser.parse_args()
print(args)

romi = AStar()

def monitor_pose():
    monitor_freq_hz = 2.0
    # move forward for two seconds
    i = 0
    while True:
        print("{:} : target_twist {:} : poses ->"
              " motor m/s {:}"
              " twist_tuple {:}".format(
            i / monitor_freq_hz,
            romi.read_twist(),
            romi.read_pose_motors(),
            romi.read_pose_twist()))
        # print motor_speeds for two seconds
        i = i + 1
        time.sleep(1.0/monitor_freq_hz)


romi.twist(args.twist_linear, args.twist_angular)

if args.twist_linear == 0.0 and args.twist_angular == 0.0:
    romi.pixels(0,0,255)
else:
    romi.pixels(0,255,0)

try:
    monitor_pose()
except (KeyboardInterrupt):
    print("all done.")
