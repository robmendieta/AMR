#!/usr/bin/env python


from planar import Point, Vec2
from planar.c import Line
from math import degrees
import rospy

#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty BugBrain class. A new instance of
#               this class will be created for each new move_to command. The
#               constructor receives the goal specification and the mode of
#               wallfollowing (left (0) or right (1)) that is currently in use.
#               All the remaining functions receive the current position and
#               orientation of the robot.
#
# Hint: you can create a class member variable at any place in your code (not
#       only in __init__) by assigning a value to it, e.g.:
#
#           self.some_member_variable = 2012
#
# Hint: you could use the 'planar' library to avoid implementing geometrical
#       functions that check the distance between a point and a line, or any
#       other helper functions that you need. To use its classes add the
#       following import statements on top of the file:
#
#            from planar import Point, Vec2
#            from planar.c import Line
#            from math import degrees
#
#       As discussed in the lab class, you will need to install the library by
#       executing `sudo pip install planar` in the terminal.
#
# Hint: all the member variables whose names start with 'wp_' (which stands for
#       'waypoint') will be automagically visualized in RViz as points of
#       different colors. Similarly, all the member variables whose names
#       start with 'ln_' (which stands for 'line') will be visualized as lines
#       in RViz. The only restriction is that the objects stored in these
#       variables should indeed be points and lines.
#       The valid points are:
#
#           self.wp_one = (1, 2)
#           self.wp_two = [1, 2]
#           self.wp_three = Point(x, y) # if you are using 'planar'
#
#       The valid lines are (assuming that p1 and p2 are valid points):
#
#           self.ln_one = (p1, p2)
#           self.ln_two = [p1, p2]
#           self.ln_three = Line.from_points([p1, p2]) # if you are using 'planar'


class BugBrain:

    POINT_TOLERANCE = 1
    TOLERANCE = 0.3

    def __init__(self, goal_x, goal_y, side):
        self.wp_goal_point = Point(goal_x,goal_y)
        self.first_line = True
        self.goal_unreachable = True
        self.pose_list = []
        pass

    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        #Create entry pose when finding wall
        self.wp_entry_pose = Point(x,y)
        self.pose_list.append(self.wp_entry_pose)
        
        #Create line only on first obstacle (line is fixed)
        if self.first_line : 
            self.ln_goal_vector = Line.from_points([self.wp_goal_point,self.wp_entry_pose])
            self.first_line = False

        # compute and store necessary variables
        pass

    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
        rospy.loginfo("leave_wall---------------------------------------")
        #Store test_pose
        self.pose_list.append(self.wp_test_pose)
        
        # compute and store necessary variables
        pass

    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        return False

    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """
        
        #rospy.loginfo("testing pose: %f  %f ", x, y)
        #rospy.loginfo("entry pose: %f  %f ", self.wp_entry_pose.x, self.wp_entry_pose.y)
        self.wp_test_pose = Point(x,y)
        distance_to_line = abs(self.ln_goal_vector.distance_to(self.wp_test_pose))
        distance_to_entry = abs(self.wp_test_pose.distance_to(self.wp_entry_pose))
        
        #If test_pose is POINT_TOLERANCE away from entry
        #AND is first time in test_pose
        #Leave wall
        if  distance_to_entry > self.POINT_TOLERANCE and self.is_pose_repeated(self.wp_test_pose) == 0:     
            if distance_to_line < self.TOLERANCE:
                rospy.loginfo("leaving wall at: %f  %f ", x, y)
                return True
                
        return False
    
    def is_pose_repeated(self, pose):
        """
        This functions returns the times the robot has gone through point pose
        """
        counter = 0
        for member in self.pose_list:
            if pose.distance_to(member) < self.POINT_TOLERANCE:
                counter = counter + 1
        return counter

#==============================================================================
