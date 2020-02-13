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
    def __init__(self):
        # open I2C port
        self.bus = smbus.SMBus(1)
        
        fw_version = self.read_firmware_version()
        if fw_version != 15:
            raise ValueError('Incorrect Romi Firmware Version Detected %d' % fw_version)

    def close(self):
        self.bus.close()

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
        return struct.unpack(format, bytes(bytearray(byte_list)))

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
    Robot Commands
    These methods send commandd to the robot 
    to take actions
    """

    def _new_twist(self):
        """set the new twist command flag high"""
        self.write_pack(12, '?', True)

    def twist(self, linear_x_m_s, angular_z_rad_s):
        """set the new twist and then set the new twist flag"""
        self.write_pack(53, 'ff', linear_x_m_s, angular_z_rad_s)
        self._new_twist()

    def read_twist(self): # mostly here for debug
        """read back the twist command sent by this driver"""
        return self.read_unpack(53, 8, 'ff')

    def leds(self, red, yellow, green):
        self.write_pack(1, '???', green, red, yellow)

    def pixels(self, red, green, blue):
        self.write_pack(4, 'BBB', red, green, blue)

    """
    Pose Related Methods
    these methods relate to estimated pose
    of the robot as measured by itself.
    """
    def read_pose_motors(self): 
        """instantaneous velocity of (left,right) wheels in m/s
        """
        left, right = self.read_unpack(45, 8, 'ff')
        return (left, right)
    
    def read_pose_twist(self):
        return self.read_unpack(37, 8, 'ff')

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
        left, right = encoder_values
        return (left, right)
        
    def read_firmware_version(self):
        return self.read_unpack(0, 1, 'B')[0]

    def read_pose_coordinate(self):
        x, y, theta = self.read_unpack(17, 12, "fff")
        return (x,y,theta)

    def read_quat(self):
        z,w = self.read_unpack(29, 8, "ff")
        return (z,w)

    def print_debug_info(self):
        print("== RomiPi Debug Info =============================")
        print(("Firmware Version: ", self.read_firmware_version()))
        print(("Battery:          ", self.read_battery_millivolts(), " mV"))
        print( "Commands:")
        print(( "Encoders (l,r):  ", self.read_encoders()))
        print(( "Commanded Twist (linear m/s, rotation rad/s: %0.1f, %0.1f" % self.read_twist()))
        print( "Estimates:")
        print(( "Estimated Twist (linear m/s, rotation rad/s): %0.2f, %0.2f" % self.read_pose_twist()))
        print(( "Estimated Pose (x m,y m, theta rad): %0.1f, %0.1f %0.1f" % self.read_pose_coordinate()))
        print(( "Estimated wheel velocity (l,r): ", self.read_pose_motors()))
        print(( "Estimated Quaternion (z,w): %0.2f, %0.2f" % self.read_quat()))

    def square(self):
        for i in range(4):
            self.twist(0.2,0.0)
            time.sleep(1.0)
            self.twist(0.0, 3.14/2)
            time.sleep(1.0)
    def line(self):
        self.twist(0.2,0.0)
        time.sleep(1.0)
        self.twist(0.0, 3.14)
        time.sleep(1.0)
        self.twist(0.2,0.0)
        time.sleep(1.0)

    def circle(self):
        self.twist(2*3.14*0.2/8,2*3.14/8)
        time.sleep(8.0)


# Self Ttest
if __name__ == '__main__':
    romi = AStar()
#    romi.reset_encoders()
    print(("Firmware Version: ", romi.read_firmware_version()))
    print(("Battery:          ", romi.read_battery_millivolts(), " mV"))
    print(("Encoders (l,r):  ", romi.read_encoders() ))
    try:
         romi.circle()
    
    except: 
        pass
    romi.twist(0,0)
