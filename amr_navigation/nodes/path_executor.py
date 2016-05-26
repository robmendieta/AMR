#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_executor'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from actionlib import SimpleActionClient, SimpleActionServer
from nav_msgs.msg import Path
from amr_msgs.msg import MoveToAction, MoveToGoal, ExecutePathAction, \
                         ExecutePathFeedback, ExecutePathResult


class PathExecutor: 
    # create messages that are used to publish feedback/result
    _feedback = ExecutePathFeedback()
    _result   = ExecutePathResult()

    def __init__(self): 
        self._as = SimpleActionServer('/path_executor/execute_path', 
            ExecutePathAction, execute_cb=self.execute_cb, 
            auto_start = False)
        self._as.start()
        rospy.loginfo("path_executor created\n")
        
        self.move_client = SimpleActionClient('/motion_controller/move_to/', MoveToAction)
        self.move_client.wait_for_server()
        #pass

    def execute_cb(self, goal):
        # helper variables
        rospy.loginfo("Goal received\n")
        r = rospy.Rate(1)
        
        #Can read goal from .action msgs
        if goal.skip_unreachable:
            rospy.loginfo("Skip unreachable = True\n")
  
        # publish the feedback
        self._as.publish_feedback(self._feedback)
        
        
        #Publish to Move_to_server
        goal2 = MoveToAction
        goal2.target_pose = goal.path.poses[0]
        self.move_client.send_goal(goal2)        
        
        r.sleep()
        #pass

    def move_to_done_cb(self, state, result):
        pass


if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()
