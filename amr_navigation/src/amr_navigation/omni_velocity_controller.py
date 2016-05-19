#!/usr/bin/env python

PACKAGE = 'amr_navigation'

from math import atan2, copysign, sin, cos, sqrt
from velocity_controller import VelocityController, Velocity
from velocity_controller import get_shortest_angle, get_distance
import rospy

class OmniVelocityController(VelocityController):

    def __init__(self, l_max_vel, l_tolerance, a_max_vel, a_tolerance, l_acceleration, a_acceleration):
        self._l_max_vel = l_max_vel
        self._l_tolerance = l_tolerance
        self._a_max_vel = a_max_vel
        self._a_tolerance = a_tolerance
        self._l_breaking_distance = (l_max_vel**2)/(2*l_acceleration)
        self._l_max_acc = l_acceleration
        self._a_breaking_distance = (a_max_vel**2)/(2*a_acceleration)
        self._a_max_acc = a_acceleration
        #raise NotImplementedError('This is your assignment to implement OmniVelocityController')
        #pass
        
    def compute_velocity(self, actual_pose):
        # Displacement and orientation to the target in world frame:
        dx = self._target_pose.x - actual_pose.x
        dy = self._target_pose.y - actual_pose.y
        dtheta = get_shortest_angle(self._target_pose.theta, actual_pose.theta)
        
        # Step 1: compute remaining distances
        linear_dist = get_distance(self._target_pose, actual_pose)
        angular_dist = get_shortest_angle(atan2(dy, dx), actual_pose.theta)
        
        #Finish status condition
        if (    abs(linear_dist)<self._l_tolerance and
                abs(dtheta)<self._a_tolerance     ):
            self._linear_complete = True
            self._angular_complete = True
            rospy.loginfo("Goal reached")
            return Velocity()
        
        # Step 2: compute velocities
    
        linear_vel_x = 0.0
        linear_vel_y = 0.0
        angular_vel = 0.0

        #compute velocity while far from goal
        if abs(linear_dist)>self._l_tolerance or abs(dtheta)>self._a_tolerance:

            #Desaccelerating before breaking distance
            if abs(linear_dist) < self._l_breaking_distance or abs(dtheta) < self._a_breaking_distance:
                linear_vel = min(sqrt(2 * linear_dist * self._l_max_acc),sqrt((2*self._a_max_acc*linear_dist**2)/abs(dtheta)))
            else:
                linear_vel = self._l_max_vel
            
            linear_vel_x = linear_vel * cos(angular_dist)
            linear_vel_y = linear_vel * sin(angular_dist)   
            
            #Angular speed relative to linear speed to allow simultaneous behavior
            angular_vel = (dtheta * linear_vel) / linear_dist
            if abs(angular_vel) > self._a_max_vel:
                angular_vel = self._a_max_vel
            
            
        return Velocity(linear_vel_x,linear_vel_y,copysign(angular_vel, dtheta))


    """
    ========================= YOUR CODE HERE =========================

    Instructions: put here all the functions that are necessary to
    implement the VelocityController interface. You may
    use the DiffVelocityController as an example.

    Implement the constructor to accept all the necessry parameters
    and implement compute_velocity() method

    You are free to write any helper functions or classes you might
    need.

    ==================================================================

    """