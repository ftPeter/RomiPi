#!/usr/bin/python2
#
# Move the robot

from romipi_astar.romipi_driver import AStar
import time
import sys

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('formation', type=str, 
                    help='move the robot in a square or circle formation')
parser.add_argument('distance', type=float, 
                    help='distance in m/s')
parser.add_argument('time', type=float, 
                    help='time in seconds')
args = parser.parse_args()
print(args)

romi = AStar()

def square():
        for i in range(args.time):
            romi.twist(args.distance,0.0)
            time.sleep(1.0)
            romi.twist(0.0, 3.14/2)
            time.sleep(1.0)


def circle():
        romi.twist(2*3.14*args.distance/args.time,2*3.14/args.time)
        time.sleep(args.time)

try:
    if args.formation == "square":
        square()
    elif args.formation == "circle":
        circle()
    else:
        print "Invalid input"
except:
	print "Unexpected error:", sys.exc_info()[0]
