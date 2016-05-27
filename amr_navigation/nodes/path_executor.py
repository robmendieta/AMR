#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_executor'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from actionlib import SimpleActionClient, SimpleActionServer
from nav_msgs.msg import Path
from amr_msgs.msg import MoveToAction, MoveToGoal, MoveToResult, ExecutePathAction, \
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
        
        

    def execute_cb(self, goal):
        #Get obstacle avoidance
        self.obstacle_avoidance = rospy.get_param('/path_executor/use_obstacle_avoidance')
        rospy.loginfo(self.obstacle_avoidance)
        
        if self.obstacle_avoidance : 
            rospy.loginfo("obstacle_avoidance = True\n")
            rospy.set_param('/motion_controller/controller','diff')
            move_client = SimpleActionClient('/bug2/move_to', MoveToAction)
        else:
            #Server from motion_controller.py, omnidrive
            move_client = SimpleActionClient('/motion_controller/move_to', MoveToAction)
            rospy.loginfo("obstacle_avoidance = False\n")
    
        move_client.wait_for_server()

        # helper variables
        rospy.loginfo("Goal received\n")
        r = rospy.Rate(10)
        
        #Can read goal from .action msgs
        if goal.skip_unreachable:
            rospy.loginfo("Skip unreachable = True\n")
  
        for index in range(len(goal.path.poses)) :            
            #Publish to Move_to server
            goal2 = MoveToAction
            goal2.target_pose = goal.path.poses[index]
            move_client.send_goal(goal2)    

            #Wait for result from move_client            
            move_client.wait_for_result()
            
            #get state
            state = move_client.get_state()
            rospy.loginfo(state)
            text = move_client.get_goal_status_text()
            rospy.loginfo(text)
            
            # publish the feedback
            self._feedback.pose = goal2.target_pose
            self._feedback.reached = True
            self._as.publish_feedback(self._feedback)

            #Store visited poses
            self._result.visited.append(True)
            r.sleep()
        
        self._as.set_succeeded(self._result)        
        #pass

    def move_to_done_cb(self, state, result):
        pass


if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()
