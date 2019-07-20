#!/usr/local/bin/python3
"""
Pose.py

Storing robot positions and
performing relative pose calculations

Peter F. Klemperer
March 24, 2018
"""

import math
from tf.transformations import quaternion_from_euler


class Pose:
    """ Store robot positions and perform relative calculations """

    def __init__(self, x=0.0, y=0.0, theta=0.0, cartesian_unit="cm", theta_unit="deg", eps=5.0):
        """

        :param x: x coordinate in cartesian_unit's
        :param y: y coordinate in cartesian_unit's
        :param theta: pose angle in theta_unit's
        :param cartesian_unit: default centimeters
        :param theta_unit: default degrees
        :param eps: maximum cartesian error for equality calculation in cm
        """
        self.x = None
        self.y = None
        self.setX(x=x, unit=cartesian_unit)
        self.setY(y=y, unit=cartesian_unit)

        self.theta = None
        if theta_unit is "Degrees" or theta_unit is "deg":
            self.setThetaDeg(theta)
        else:
            self.setThetaRad(theta)
        self.eps = eps

    def getCartesianPoseDeg(self):
        return self.getX(), self.getY(), self.getThetaDeg()

    def getCartesianPoseRad(self):
        return self.getX(), self.getY(), self.getThetaRad()

    def getPolarPoseDeg(self):
        rho, psi_rad, theta_rad = self.getPolarPoseRad()
        return rho, math.degrees(psi_rad), math.degrees(theta_rad)

    def getPolarPoseRad(self):
        rho = math.sqrt(self.getX() * self.getX() + self.getY() * self.getY())
        psi = math.atan2(self.getY(), self.getX())
        theta = self.getThetaRad()
        return rho, psi, theta

    @staticmethod
    def cm_to_m(cm):
        return cm / 100.0

    @staticmethod
    def m_to_cm(m):
        return m * 100.0

    def getX(self, unit="cm"):
        if unit is "cm":
            return self.x
        elif unit is "m":
            return Pose.cm_to_m(self.x)
        else:
            return None

    def setX(self, x=None, unit="cm"):
        if unit is "cm":
            self.x = x
        elif unit is "m":
            self.x = Pose.m_to_cm(x)

    def getY(self, unit="cm"):
        if unit is "cm":
            return self.y
        elif unit is "m":
            return Pose.cm_to_m(self.y)
        else:
            return None

    def setY(self, y=None, unit="cm"):
        if unit is "cm":
            self.y = y
        elif unit is "m":
            self.y = Pose.cm_to_m(y)

    def getTheta(self, unit="deg"):
        if unit is "deg" or unit is "Degrees":
            return math.degrees(self.theta)
        elif unit is "rad" or unit is "Radians":
            return self.theta
        else:
            return None

    def getThetaDeg(self):
        return math.degrees(self.theta)

    def getThetaRad(self):
        return self.theta

    def setTheta(self, theta, unit="deg"):
        if unit is "deg" or unit is "Degrees":
            self.theta = math.radians(theta)
        elif unit is "rad" or unit is "Radians":
            self.theta = theta
        else:
            self.theta = None

    def setThetaRad(self, theta_rad):
        self.theta = self._principal_angle(theta_rad)

    def setThetaDeg(self, theta_deg):
        self.setThetaRad(math.radians(theta_deg))

    def getQuaternion(self):
        q = quaternion_from_euler(0.0, 0.0, self.getThetaRad())
        return q

    def getRange(self):
        return math.sqrt(self.getX() * self.getX() + self.getY() * self.getY())

    def calculateDistance(self, other):
        x = self.getX() - other.getX()
        y = self.getY() - other.getY()
        return math.sqrt(x*x+y*y)

    @staticmethod
    def _principal_angle(theta_rad):
        if theta_rad == -1 * math.pi:
            theta_rad = math.pi
        return math.atan2(math.sin(theta_rad), math.cos(theta_rad))

    def _close(self, a, b):
        if abs(a - b) > self.eps:
            print("{} != {}".format(a, b))
            return False
        return True

    def _closePose(self, left, right):
        if len(left) != len(right):
            return False

        rx,ry,right_angle = right
        lx,ly,left_angle = left

        if abs(rx-lx) > self.eps:
            return False
        if abs(ry-ly) > self.eps:
            return False

        # need to compare pose angles to make sure that
        # the search functions work
        if abs(right_angle-left_angle) > (math.pi/100.0):
            return False
        return True

    def __eq__(self, other):
        if isinstance(other, Pose):
            return self._closePose(self.getCartesianPoseRad(), other.getCartesianPoseRad())
        return NotImplemented

    def __str__(self):
        return "Pose ({:.1f},{:.1f},{:.1f}) XYT cm, deg".format(self.getX(), self.getY(), self.getThetaDeg())

    @staticmethod
    def mean_pose(pose_list):
        x = 0
        y = 0
        theta = 0
        for pose in pose_list:
            x += pose.getX()
            y += pose.getY()
            theta += pose.getTheta()
        return Pose(x,y,theta)

# TODO write unit test for PAB Pose
class PABPose(Pose):
    def __init__(self, rho=0.0, a=0.0, b=0.0, eps=0.001, alpha_unit="Radians", beta_unit="Radians"):
        if alpha_unit is "Radians":
            a_rad = a
        else:
            a_rad = math.radians(a)

        if beta_unit is "Radians":
            b_rad = b
        else:
            b_rad = math.radians(b)

        x = rho * math.cos(a)  # TODO this code is total non-sense, doesn't work
        y = rho * math.sin(a)  # TODO and needs to be re-written.
        theta_deg = a_rad + b_rad

        super(PABPose, self).__init__(x=x, y=y, theta=theta_deg, theta_unit="Degrees", eps=eps)


class PolarPose(Pose):
    def __init__(self, rho=0.0, psi=0.0, theta=0.0, eps=0.001, psi_unit="Degrees", theta_unit="Degrees"):
        if psi_unit is "Degrees":
            psi_rad = math.radians(psi)
        else:
            psi_rad = psi
        x = rho * math.cos(psi_rad)
        y = rho * math.sin(psi_rad)
        super(PolarPose, self).__init__(x=x, y=y, theta=theta, theta_unit=theta_unit, eps=eps)

import unittest
class TestPose(unittest.TestCase):
    @staticmethod
    def closeTuple(abc, cde):
        if len(abc) != len(cde):
            return False
        for i, j in zip(abc, cde):
            if abs(i - j) > 0.001:
                print(abc, cde)
                print("{} != {}".format(i, j))
                return False
        return True

    def test_pose_constructor(self):
        self.assertTrue(Pose(0.0, 0.0, 0.0) == Pose(0, 0.0, 0.0))
        self.assertTrue(Pose(0.0, 0.0, 0.0) == Pose(0, 0.0, 360.0))
        self.assertTrue(Pose(0.0, 0.0, 180.0) == Pose(0, 0.0, 180.0))
        self.assertTrue(Pose(0.0, 0.0, -180.0) == Pose(0, 0.0, 180.0))
        self.assertTrue(Pose(0.0, 0.0, 0.0) == Pose(0, 0.0, 0.0))
        self.assertTrue(Pose(0.0, 0.0, 0.0) == Pose(0, 0.0, 360.0))
        self.assertTrue(Pose(0.0, 0.0, 90.0) == Pose(0, 0.0, -270.0))
        self.assertTrue(Pose(0.0, 0.0, -45.0) == Pose(0, 0.0, 360.0 - 45))
        self.assertTrue(Pose(0.0, 0.0, math.pi, theta_unit="Rad") == Pose(0, 0.0, 180))
        return

    def test_getPolarPose(self):
        self.assertTrue(self.closeTuple(Pose(0.0, 0.0, 0.0).getPolarPoseDeg(), (0, 0, 0)))
        self.assertTrue(self.closeTuple(Pose(1.0, 0.0, 0.0).getPolarPoseDeg(), (1, 0, 0)))
        self.assertTrue(self.closeTuple(Pose(-1.0, 0.0, 0.0).getPolarPoseDeg(), (1, 180, 0)))
        self.assertTrue(self.closeTuple(Pose(0, 1.0, 0.0).getPolarPoseDeg(), (1, 90, 0)))
        self.assertTrue(self.closeTuple(Pose(0, -1.0, 0.0).getPolarPoseDeg(), (1, -90, 0)))

    def test_PolarPose(self):
        self.assertTrue(PolarPose(0.0, 0.0, 0.0) == Pose(0, 0.0, 0.0))
        self.assertTrue(PolarPose(1.0, 0.0, 0.0) == Pose(1, 0.0, 0.0))
        self.assertTrue(PolarPose(1.0, 180.0, 0.0) == Pose(-1, 0.0, 0.0))
        self.assertTrue(PolarPose(1.0, -180.0, 0.0) == Pose(-1, 0.0, 0.0))
        self.assertTrue(PolarPose(1.0, -180.0, 90.0) == Pose(-1, 0.0, 90.0))


# Self Test
if __name__ == '__main__':
    import unittest
    print("UNIT TESTING: TARGETING.PY BEGIN")
    unittest.main()
