#!/usr/bin/env python

PACKAGE = 'amr_navigation'

from geometry_msgs.msg import Twist, Pose2D
from exceptions import NotImplementedError
import math


class VelocityController:
    """
    Base class for velocity controller for the sake of OOP
    """
    
    def __init__(self, *args):
        self._linear_complete  = True
        self._angular_complete = True
        pass


    def set_target_pose(self, pose):
        self._target_pose = pose
        self._linear_complete = False
        self._angular_complete = False


    def is_target_reached(self):
        return self._linear_complete and self._angular_complete


    def compute_velocity(self, *args):
        raise NotImplementedError('Subclasses should implement compute_velocity() method!')



class Velocity:
    """
    x-y-theta wrapper of the Twist
    """
    
    def __init__(self, x = 0, y=0, theta = 0):
        self.x, self.y, self.theta = x, y, theta


    def get_twist(self):
        """
        returns Twist object representing the velocity
        """
        twist = Twist()
        twist.linear.x = self.x
        twist.linear.y = self.y
        twist.angular.z = self.theta
        return twist


#helper functions
def get_distance(pose1, pose2):
    """
    Computes euclidian distance between two given poses (Pose2D)
    """
    return math.sqrt((pose1.x-pose2.x)**2+(pose1.y-pose2.y)**2)


def get_shortest_angle(target_angle, current_angle):
    """
    Helper function to compute the angular distance between two angles (in radians)
    """
    a1 = target_angle
    a2 = current_angle
    return math.atan2(math.sin(a1-a2), math.cos(a1-a2))

