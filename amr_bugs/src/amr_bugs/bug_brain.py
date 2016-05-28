#!/usr/bin/env python


from planar import Point, Vec2
from planar.c import Line
from math import degrees, sin, cos
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
    
    #Tolerance for distance between points
    POINT_TOLERANCE = 0.5
    #Tolerance for line detection
    TOLERANCE = 0.3

    def __init__(self, goal_x, goal_y, side):
        self.wp_goal_point = Point(goal_x,goal_y)
        self.first_line = True
        self.goal_unreachable = True
        self.pose_list = []
        self.old_is_left = True
        self.new_is_left = True

        pass

    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        #Create entry pose when finding wall 
        self.wp_entry_pose = Point(x,y)
          
        #Create line only on first obstacle encountered
        if self.first_line : 
            self.ln_goal_vector = Line.from_points([self.wp_goal_point,self.wp_entry_pose])
            self.first_line = False
                
        self.old_is_left = self.ln_goal_vector.point_left(self.wp_entry_pose)
        
        
        pass

    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
           
        pass

    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        
        if self.is_pose_repeated(Point(x,y)) > 1:
            return True
        
        return False

    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """
        
        self.wp_test_pose = Point(x,y)
        self.new_is_left = self.ln_goal_vector.point_left(self.wp_test_pose)
        
        #Line for comparing if robot is to left
        self.ln_robot = Line.from_points([Point(x,y),Point(x+cos(theta),y+sin(theta))])
      
        #Check for ditance to entry point
        if abs(self.wp_test_pose.distance_to(self.wp_entry_pose)) > self.POINT_TOLERANCE:
            #Check for a change on side            
            if self.old_is_left != self.new_is_left:
                self.old_is_left = self.new_is_left            
                
                #Save point
                self.pose_list.append(self.wp_test_pose)
                
                #Check goal is left to robot
                if self.ln_robot.distance_to(self.wp_goal_point) < 0:
                    #Check if first time on point, then leave                      
                    if self.is_pose_repeated(self.wp_test_pose) == 1:
                        return True
                        
        return False
    
    def is_pose_repeated(self, pose):
        """
        This functions returns the times the robot has gone through point pose
        """
        counter = 0
        for member in self.pose_list:
            if abs(pose.distance_to(member)) < self.POINT_TOLERANCE:
                counter = counter + 1
        return counter

#==============================================================================
