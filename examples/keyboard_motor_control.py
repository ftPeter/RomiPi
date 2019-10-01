#!/usr/bin/python2
#
# Move the robot forwards and backwards
from romipi_astar.romipi_driver import AStar
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
    print "UPDATED TWIST CMD: linear m/s %0.1f, rotation rad/s %0.1f" % (linear_ms, rotate_rads)

def update_twist():
    """send twist to driver and change light color if moving"""
    if linear_ms == 0.0 and rotate_rads == 0.0:
        romi.pixels(0, 0, 100)
    else:
        romi.pixels(0, 100, 0)
    romi.twist(linear_ms, rotate_rads)

# all set up, now run the robot
print_instructions()
while True:
    c = getch()
    if c.upper() == 'W': # forward
        linear_ms += 0.1
    elif c.upper() == 'S': # back
        linear_ms -= 0.1
    elif c.upper() == 'A': # left
        rotate_rads += 3.14 * 2.0 / 8.0
    elif c.upper() == 'D': # right
        rotate_rads -= 3.14 * 2.0 / 8.0
    elif c.upper() == 'Z': # halt
        linear_ms = 0.0
        rotate_rads = 0.0
    elif c.upper() == 'I':
        romi.print_debug_info()
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
