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
    def __init__(self): 
        #Create SimpleActionServer
        self._as = SimpleActionServer('/path_executor/execute_path', 
            ExecutePathAction, execute_cb=self.execute_cb, 
            auto_start = False)
        rospy.loginfo("/path_executor/execute_path action served created")        
        self._as.start()
        rospy.loginfo("/path_executor/execute_path action served started...\n")        
        
        

    def execute_cb(self, goal):
        rospy.loginfo("ExecutePathAction received")         
        # create messages that are used to publish feedback/result
        _feedback = ExecutePathFeedback()
        _result   = ExecutePathResult()
        r = rospy.Rate(10)
        success = True
        aborted = False
        goal2 = MoveToAction
        
        #Get obstacle avoidance
        self.obstacle_avoidance = rospy.get_param('/path_executor/use_obstacle_avoidance')
  
        #Choose server for move_to depending on obstacle avoidance
        if self.obstacle_avoidance : 
            rospy.loginfo("obstacle_avoidance = True")
            move_client = SimpleActionClient('/bug2/move_to', MoveToAction)
        else:
            rospy.loginfo("obstacle_avoidance = False")                
            move_client = SimpleActionClient('/motion_controller/move_to', MoveToAction)

        move_client.wait_for_server()            
            
        #Iterate through path
        for index in range(len(goal.path.poses)) :  
            rospy.loginfo("\nGoal received")         
            rospy.loginfo("Goal X: %f", goal.path.poses[index].pose.position.x)
            rospy.loginfo("Goal Y: %f", goal.path.poses[index].pose.position.y)
            rospy.loginfo("Goal Yaw: %f", goal.path.poses[index].pose.orientation.z)
            
            #Publish goal pose to move_to server
            goal2.target_pose = goal.path.poses[index]
            move_client.send_goal(goal2)  
            rospy.loginfo("Sending goal to move_to action server ...")
                
            while not rospy.is_shutdown() :
                #Check for preemption
                if self._as.is_preempt_requested():
                    rospy.logwarn('Preempted requested')
                    move_client.cancel_all_goals()
                    self._as.set_preempted()                    
                    success = False
                    aborted = True
                    break
                
                #get result state from move_to server
                state = move_client.get_state()

                #Success
                if state == 3:
                    rospy.loginfo("Goal reached")
                    #Publish the feedback
                    _feedback.pose = goal2.target_pose
                    _feedback.reached = True
                    self._as.publish_feedback(_feedback)
                    #Store to visited poses
                    _result.visited.append(True)
                    break
                #Unreachable goal
                elif state == 4:
                    rospy.logwarn("Goal is unreachable")
                    #Skip unreachable
                    if goal.skip_unreachable:
                        rospy.logwarn("Skipping unreachable goal")
                        #Publish the feedback
                        _feedback.pose = goal2.target_pose
                        _feedback.reached = False
                        self._as.publish_feedback(_feedback)
                        #Store visited poses
                        _result.visited.append(False)
                        break
                    #Abort                    
                    else:
                        rospy.logwarn("Abort unreachable goal")
                        rospy.loginfo("Cancelling goals...")
                        move_client.cancel_all_goals()
                        rospy.logwarn("Goals cancelled")
                        rospy.loginfo("Aborting action server")
                        self._as.set_aborted()      
                        rospy.logwarn("Action server aborted")
                        success = False
                        aborted = True
                        break
                    
                r.sleep()
                
            if aborted:
                break
                
        
        if success:        
            self._as.set_succeeded(_result)
            rospy.loginfo("Path finished with success\n")
                    

    def move_to_done_cb(self, state, result):
        pass


if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()
