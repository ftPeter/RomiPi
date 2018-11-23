#!/usr/bin/python2
#
# Move the robot forwards and backwards
from romipi_driver import AStar
import time

romi = AStar()
linear_ms = 0.0
rotate_rads = 0.0

def getch():
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def print_instructions():
    print "press w,a,s,d,z to move the robot, q to quit:"

def print_speed():
    print "linear m/s %0.1f, rotation rad/s %0.1f" % (linear_ms, rotate_rads)

def update_twist():
    romi.twist(linear_ms, rotate_rads)
    if linear_ms == 0.0 and rotate_rads == 0.0:
        romi.pixels(0, 0, 100)
    else:
        romi.pixels(0, 100, 0)

# all set up, now run the robot
print_instructions()
while True:
    c = getch()
    if c.upper() == 'W':
        linear_ms += 0.1
    elif c.upper() == 'A':
        rotate_rads -= 0.1
    elif c.upper() == 'S':
        linear_ms = 0.0
        rotate_rads = 0.0
    elif c.upper() == 'D':
        rotate_rads += 0.1
    elif c.upper() == 'Z':
        linear_ms -= 0.1
    elif c.upper() == 'Q':
        break
    else:
        print_instructions()
        continue
    print_speed()
    update_twist()

# stop motors and shut down light
romi.twist(0.0, 0.0)
romi.pixels(0, 0, 0)
