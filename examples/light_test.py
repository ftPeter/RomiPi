#!/usr/bin/python2
#
# Blink the lights connected to the RomiPi
# The neopixel should turn red, yellow, then red
from romipi_astar.romipi_driver import AStar
import time

led_delay_s = 2.0

romi = AStar()

# turn on red LED
romi.leds(True,False,False)
romi.pixels(250,0,0)
print("LED RED")
print("PIXEL RED")
time.sleep(led_delay_s)

# turn on yellow LED
romi.leds(False,True,False)
romi.pixels(0,250,0)
print("LED YELLOW")
print("PIXEL GREEN")
time.sleep(led_delay_s)

# turn on green LED
romi.leds(False,False,True)
romi.pixels(0,0,250)
print("LED GREEN")
print("PIXEL BLUE")
time.sleep(led_delay_s)

# turn off LED
romi.leds(False,False,False)
romi.pixels(0,0,0)
print("LED OFF")
print("PIXEL OFF")

