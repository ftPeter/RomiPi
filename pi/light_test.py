#!/usr/bin/python3
#
# Blink the lights connected to the RomiPi
# The neopixel should turn red, yellow, then red
from a_star import AStar
import time

led_delay_s = 2.0

romi = AStar()

# turn on red LED
romi.leds(True,False,False)
print("LED RED")
time.sleep(led_delay_s)

# turn on yellow LED
romi.leds(False,True,False)
print("LED YELLOW")
time.sleep(led_delay_s)

# turn on green LED
romi.leds(False,False,True)
print("LED GREEN")
time.sleep(led_delay_s)

# turn off LED
romi.leds(False,False,False)
print("LED OFF")
time.sleep(led_delay_s)

