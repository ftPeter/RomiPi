#!/usr/bin/python2
#
# Print out the pose estimates
# without setting twist so that the
# user can push the robot around

from romipi_astar.romipi_driver import AStar
import time

romi = AStar()

try:
    romi.reset_encoders()

    while True:
        romi.print_debug_info()
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
except Exception as e:
    print e

romi.close()
