#!/usr/bin/python2
#
# Notes:
# read_unpack, write_pack types quick reference
# https://docs.python.org/3.6/library/struct.html#format-characters
# ? - bool           - 1
# B - unsigned char  - 1
# h - short          - 2
# H - unsigned short - 2
# f - float          - 4
#
# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

import smbus
import struct
import time

class AStar:
    def __init__(self, left_m = -1, right_m = -1, swap_m = False, left_e = -1, right_e = -1, swap_e = True):
        # configure motors
        self.flip_left_motor = left_m
        self.flip_right_motor = right_m
        self.swap_motors = swap_m
        # configure encoders
        self.flip_left_encoder = left_e
        self.flip_right_encoder = right_e
        self.swap_encoders = swap_e
        # open I2C port
        self.bus = smbus.SMBus(1)

    def close(self):
        self.bus.close()

    def flip_motors(self, left, right ):
      self.flip_left_motor  = left if -1 else 1
      self.flip_right_motor = right if -1 else 1

    def swap_motors(self, swap ):
      self.swap_motors = swap

    def flip_encoders(self, left, right ):
      self.flip_left_motor  = left if -1 else 1
      self.flip_right_motor = right if -1 else 1

    def swap_encoders(self, swap ):
      self.swap_encoders = swap

    def read_raw(self, size):
        try:
            byte_list = [self.bus.read_byte(20) for _ in range(size)]
        except IOError:
            print("IOError Detected: read_raw")
            return None
        return byte_list


    def read_unpack(self, address, size, format):
        # Ideally we could do this:
        #    byte_list = self.bus.read_i2c_block_data(20, address, size)
        # But the AVR's TWI module can't handle a quick write->read transition,
        # since the STOP interrupt will occasionally happen after the START
        # condition, and the TWI module is disabled until the interrupt can
        # be processed.
        #
        # A delay of 0.0001 (100 us) after each write is enough to account
        # for the worst-case situation in our example code.
        self.bus.write_byte(20, address)
        time.sleep(0.0002)
        byte_list = [self.bus.read_byte(20) for _ in range(size)]
        return struct.unpack(format, bytes(byte_list))

    def write_pack(self, address, format, *data):
        for i in range(2):
            try:
                data_array = list(struct.pack(format, *data))
                self.bus.write_i2c_block_data(20, address, data_array)
            except IOError:
                write_fail_flag = True
                print("IOError Detected: write_pack")
                continue
            break
        time.sleep(0.0001)

    """ 
    read_twist

    mostly for debugging!
    read back the twist sent by this driver
    """
    def read_twist(self):
        return self.read_unpack(17, 8, 'ff')

    def twist(self, linear_x_m_s, angular_z_rad_s):
        twist_tuple = self.read_twist()
        #print("twist is {:}".format(twist_tuple))
        self.write_pack(17, 'f', linear_x_m_s)
        self.write_pack(21, 'f', angular_z_rad_s)

    def leds(self, red, yellow, green):
        self.write_pack(1, '???', green, red, yellow)

    def pixels(self, red, green, blue):
        self.write_pack(4, 'BBB', red, green, blue)

    def motors(self, left, right):
        print("ERROR, to set motor speed, use twist instead")
        print("       in future, this class should not inherit")
        print("       the legacy romi HWBase")
        return (0, 0)

    """
    read_motors_pose
    returns the instantaneous velocity of each
    wheel in meters per second.
    """
    def read_pose_motors(self):
        if self.swap_motors:
            left, right = self.read_unpack(49, 8, 'ff')
        else:
            right,left = self.read_unpack(49, 8, 'ff')
        return (left, right)

    def read_pose_twist(self):
        return self.read_unpack(41, 8, 'ff')

    def read_buttons(self):
        return self.read_unpack(7, 3, "???")

    def read_battery_millivolts(self):
        return self.read_unpack(10, 2, "H")[0]

    def read_analog(self):
        print ("ERR: disabled")
        return ()

    def read_encoders(self):
        encoder_values = self.read_unpack(13, 4, 'hh')
        if( encoder_values is None ):
            return (None,None)
        elif self.swap_encoders:
            right, left = encoder_values
        else:
            left, right = encoder_values
        return self.flip_left_encoder * left, self.flip_left_encoder * right

    def read_firmware_version(self):
        return self.read_unpack(0, 1, 'B')[0]

    def reset_encoders(self):
        # set the reset bit high
        self.write_pack(12, '?', 1)

# Self Test
if __name__ == '__main__':
    romi = AStar()
    romi.reset_encoders()
    print("Firmware Version: ", romi.read_firmware_version() )
    print("Battery:          ", romi.read_battery_millivolts(), " mV")
    print("Encoders (l,r):  ", romi.read_encoders() )
    romi.twist(0.5, 0.0)
    while True:
        print("Encoders (l,r):  ", romi.read_encoders() )
        print("Motor Targets (l,r):", romi.read_motors() )
        time.sleep(0.5)
