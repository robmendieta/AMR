#!/usr/bin/env python

"""
This module provides a single construct() function which produces a Smach state
machine that implements wallfollowing behavior.

The state machine contains three states:
    * SEARCH     initial state - drives until a wall is detected
    * ALLIGN     aligning state - used to align parallel to wall and curls around concave corners
    * FOLLOW    following state - used to follow a straight wall
    * WALL    following state - used to turn out of convex corners

The constructed state machine has three attached methods:
    * set_ranges(ranges): this function should be called to update the range
                          readings
    * get_twist(): returns a twist message that could be directly passed to the
                   velocity publisher
    * set_config(config): updates the machine userdata with the new config

The constructed state machine is preemptable, i.e. each state checks whether
a preemption is requested and returns 'preempted' if that is the case.
"""

PACKAGE = 'amr_bugs'

import math
import roslib
roslib.load_manifest(PACKAGE)
import smach
from preemptable_state import PreemptableState
from types import MethodType
from geometry_msgs.msg import Twist


__all__ = ['construct']

#=============================== YOUR CODE HERE ===============================
# Instructions: write a function for each state of wallfollower state machine.
#               The function should have exactly one argument (userdata
#               dictionary), which you should use to access the input ranges
#               and to provide the output velocity.
#               The function should have at least one 'return' statement, which
#               returns one of the possible outcomes of the state.
#               The function should not block (i.e. have infinite loops), but
#               rather it should implement just one iteration (check
#               conditions, compute velocity), because it will be called
#               regularly from the state machine.
#
# Hint: below is an example of a state that moves the robot forward until the
#       front sonar readings are less than the desired clearance. It assumes
#       that the corresponding variables ('front_min' and 'clearance') are
#       available in the userdata dictionary.
#
#           def search(ud):
#               if ud.front_min < ud.clearance:
#                   return 'found_obstacle'
#               ud.velocity = (1, 0, 0)
#==============================================================================


def search(userdata):
    #Go forward, on wall < clearance go to WALL state 
    userdata.velocity = (userdata.max_forward_velocity, 0, 0)

    if userdata.all_min < userdata.clearance:
        return 'found_wall'


def allign(userdata):
    #Rotate to keep side_balance close to 0
    #Translate sideways for side_avg_distance => clearance
    
    clearance_tolerance = 0.1

    #Error between clearance and side distance
    clearance_error = userdata.clearance - userdata.side_avg_distance
    side_velocity = math.copysign(userdata.max_forward_velocity, clearance_error)  
    
    #Change direction for mode = 0
    if userdata.mode == 0:
        side_velocity = -side_velocity

    #Velocity
    userdata.velocity = (0, side_velocity, 0)

    #If alligned, got to state FOLLOW
    #if abs(userdata.side_balance) < angular_tolerance and abs(clearance_error) < clearance_tolerance :
    if abs(clearance_error) < clearance_tolerance :
        return 'is_alligned'

def follow(userdata):
    #Go straight
    angular_tolerance = 0.05
    clearance_tolerance = 0.1
    
    #Angular velocity depends on difference between side sensors
    if abs(userdata.side_balance) > angular_tolerance:
        angular_velocity = math.copysign(userdata.default_rotational_speed,userdata.side_balance)
    else:
        angular_velocity = 0
    
    #Error between clearance and side distance
    clearance_error = userdata.clearance - userdata.side_avg_distance
    
    #Velocity
    userdata.velocity = (userdata.max_forward_velocity ,0 ,angular_velocity)
    
    #if abs(userdata.side_balance) > angular_tolerance or abs(clearance_error) > clearance_tolerance:
    if abs(clearance_error) > clearance_tolerance:
        return 'is_not_alligned'
    if userdata.front_min < userdata.clearance:
        return 'avoid_wall' 

 
def wall(userdata):
    #Rotate until sensors on front > clearance
    if userdata.mode == 1:
        userdata.velocity = (0, 0, userdata.default_rotational_speed)
    elif userdata.mode == 0:
        userdata.velocity = (0, 0, -userdata.default_rotational_speed)
    
    if userdata.front_min > 1.5 * userdata.clearance:
        return 'wall_avoided'
        
       
    
def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback. 
    For left hand side wallfollowing, the sensor values are mirrored (sides are swapped).
    """
    side_min1 = min(ranges[8].range, ranges[7].range, ranges[6].range)
    side_min0 = min(ranges[15].range, ranges[0].range, ranges[1].range)
    back_min = min(ranges[9].range, ranges[10].range, ranges[11].range, ranges[12].range, ranges[13].range, ranges[14].range)

    if self.userdata.mode == 1:
        self.userdata.front_min = min(ranges[1].range, ranges[6].range, ranges[2].range, ranges[3].range, ranges[4].range, ranges[5].range)
        self.userdata.side_balance = ranges[8].range -  ranges[7].range
        self.userdata.side_avg_distance = min(ranges[8].range, ranges[7].range)
        self.userdata.corner = ranges[6].range
    
    elif self.userdata.mode == 0:
        self.userdata.front_min = min(ranges[1].range, ranges[6].range, ranges[2].range, ranges[3].range, ranges[4].range, ranges[5].range)
        self.userdata.side_balance = ranges[0].range -  ranges[15].range
        self.userdata.side_avg_distance = min(ranges[15].range, ranges[0].range)
    
    self.userdata.all_min = min(side_min1, side_min0, back_min, self.userdata.front_min)
    self.userdata.width = side_min1 + side_min0

    #============================= YOUR CODE HERE =============================
    # Instructions: store the ranges from a ROS message into the userdata
    #               dictionary of the state machine.
    #               'ranges' is a list or Range messages (that should be
    #               familiar to you by now). It implies that to access the
    #               actual range reading of, say, sonar number 3, you need to
    #               write:
    #
    #                   ranges[3].range
    #
    #               For example, to create an item called 'front_min', which
    #               contains the minimum between the ranges reported by the two
    #               front sonars, you would write the following:
    #
    #                   self.userdata.front_min = min(ranges[3].range, ranges[4].range)
    #
    # Hint: you can just store the whole array of the range readings, but to
    #       simplify the code in your state functions, you may compute
    #       additional values, e.g. the difference between the reading of the
    #       side sonars, or the minimum of all sonar readings, etc.
    #
    # Hint: you can access all the variables stored in userdata. This includes
    #       the current settings of the wallfollower (that is clearance and the
    #       mode of wallfollowing). Think about how you could make your state
    #       functions independent of wallfollowing mode by smart preprocessing
    #       of the sonar readings.
    #==========================================================================
    

def get_twist(self):
    """
    This function will be attached to the constructed wallfollower machine.
    It creates a Twist message that could be directly published by a velocity
    publisher. The values for the velocity components are fetched from the
    machine userdata.
    """
    twist = Twist()
    twist.linear.x = self.userdata.velocity[0]
    twist.linear.y = self.userdata.velocity[1]
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = self.userdata.velocity[2]

    #============================= YOUR CODE HERE =============================
    # Instructions: although this function is implemented, you may need to
    #               slightly tweak it if you decided to handle wallfolllowing
    #               mode in "the smart way".
    # Hint: state machine userdata is accessible in this function as well, for
    #       example you can read the current wallfollowing mode with
    #
    #           self.userdata.mode
    #
    #==========================================================================

    return twist


def set_config(self, config):
    """
    This function will be attached to the constructed wallfollower machine.
    It updates the relevant fields in the machine userdata.
    Its argument is the config object that comes from ROS dynamic reconfigure
    client. self.userdata.direction sets a velocity sign depending on the mode.
    """
    self.userdata.mode = config['mode']
    self.userdata.clearance = config['clearance']
    if self.userdata.mode == 1:
        self.userdata.direction = 1
    else:
        self.userdata.direction = -1
    return config


def construct():
    sm = smach.StateMachine(outcomes=['preempted'])
    # Attach helper functions
    sm.set_ranges = MethodType(set_ranges, sm, sm.__class__)
    sm.get_twist = MethodType(get_twist, sm, sm.__class__)
    sm.set_config = MethodType(set_config, sm, sm.__class__)
    # Set initial values in userdata
    sm.userdata.velocity = (0, 0, 0)
    sm.userdata.mode = 1
    sm.userdata.clearance = 0.3
    sm.userdata.ranges = None
    sm.userdata.max_forward_velocity = 0.3
    sm.userdata.default_rotational_speed = 0.5
    sm.userdata.direction = 1
    
    with sm:
        smach.StateMachine.add('SEARCH',
                                                  PreemptableState(search,
                                                                   input_keys=['mode',
                                                                   'all_min',
                                                                   'clearance',
                                                                   'max_forward_velocity'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['found_wall']),
                                                 transitions={'found_wall': 'WALL'})
        
        smach.StateMachine.add('ALLIGN',
                                                  PreemptableState(allign,
                                                                   input_keys=['mode',
                                                                               'width',
                                                                   'side_balance',
                                                                    'side_avg_distance',
                                                                   'default_rotational_speed',
                                                                   'clearance',
                                                                   'max_forward_velocity',
                                                                   'corner',
                                                                   'front_min'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['is_alligned']),
                                                    transitions={'is_alligned': 'FOLLOW'})
        smach.StateMachine.add('FOLLOW',
                                                  PreemptableState(follow,
                                                                   input_keys=['mode',
                                                                               'width',
                                                                   'front_min',
                                                                    'clearance',
                                                                   'max_forward_velocity',
                                                                   'side_balance',
                                                                    'side_avg_distance',
                                                                   'default_rotational_speed',
                                                                   'corner'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['avoid_wall',
                                                                             'is_not_alligned']),
                                                    transitions={'avoid_wall':'WALL',
                                                                 'is_not_alligned':'ALLIGN'})   
        smach.StateMachine.add('WALL',
                                                  PreemptableState(wall,
                                                                   input_keys=['mode',
                                                                   'default_rotational_speed',
                                                                   'clearance',
                                                                   'front_min'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['wall_avoided']),
                                                    transitions={'wall_avoided': 'FOLLOW'})
                                                   

                                                    
        #=========================== YOUR CODE HERE ===========================
        # Instructions: construct the state machine by adding the states that
        #               you have implemented.
        #               Below is an example how to add a state:
        #
        #                   smach.StateMachine.add('SEARCH',
        #                                          PreemptableState(search,
        #                                                           input_keys=['front_min', 'clearance'],
        #                                                           output_keys=['velocity'],
        #                                                           outcomes=['found_obstacle']),
        #                                          transitions={'found_obstacle': 'ANOTHER_STATE'})
        #
        #               First argument is the state label, an arbitrary string
        #               (by convention should be uppercase). Second argument is
        #               an object that implements the state. In our case an
        #               instance of the helper class PreemptableState is
        #               created, and the state function in passed. Moreover,
        #               we have to specify which keys in the userdata the
        #               function will need to access for reading (input_keys)
        #               and for writing (output_keys), and the list of possible
        #               outcomes of the state. Finally, the transitions are
        #               specified. Normally you would have one transition per
        #               state outcome.
        #
        # Note: The first state that you add will become the initial state of
        #       the state machine.
        #======================================================================
    
    
    return sm
