from abc import ABC, abstractmethod

class HWBase(ABC):
    def __init__(self, left_m=-1, right_m=-1, swap_m=False, left_e=-1, right_e=-1, swap_e=True):
        # configure motors
        self.flip_left_motor = left_m
        self.flip_right_motor = right_m
        self.swap_motors = swap_m
        # configure encoders
        self.flip_left_encoder = left_e
        self.flip_right_encoder = right_e
        self.swap_encoders = swap_e

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def motor_velocities(self, left, right):
        left  = self.flip_left_motor * left
        right = self.flip_right_motor * right

    @abstractmethod
    def read_encoders(self):
        encoder_values = (0,0)
        if (encoder_values is None):
            return (None, None)
        elif self.swap_encoders:
            right, left = encoder_values
        else:
            left, right = encoder_values
        return self.flip_left_encoder * left, self.flip_left_encoder * right

    @abstractmethod
    def reset_encoders(self):
        pass

    @abstractmethod
    def read_firmware_version(self):
        pass

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
