#!/usr/bin/python2
# pixels_utility.py
#
# Manually control the Neopixel colors
# on the Romi
# 
# Peter F. Klemperer
# September 26, 2018
from romipi_astar.romipi_driver import AStar
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('red', type=int, 
                    help='red (0-255) value of pixel')
parser.add_argument('green', type=int, 
                    help='green (0-255) value of pixel')
parser.add_argument('blue', type=int, 
                    help='blue (0-255) value of pixel')
args = parser.parse_args()
print(args)

romi = AStar()

# turn on pixel according to arguments
romi.pixels(args.red, args.green, args.blue)

