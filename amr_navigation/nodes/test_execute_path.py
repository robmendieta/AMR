#!/usr/bin/env python
PACKAGE = 'amr_navigation'
NODE = 'test_execute_path'



import roslib
roslib.load_manifest(PACKAGE)
import rospy
import numpy as np
import sys
import argparse
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
from amr_msgs.msg import ExecutePathGoal, ExecutePathAction


def to_str(pose):
    rpy = euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w))
    return '%.3f %.3f %.3f' % (pose.position.x, pose.position.y, rpy[2])


def feedback_cb(feedback):
    print ' -> %s %s' % ('visited' if feedback.reached else 'skipped',
                         to_str(feedback.pose.pose))


def done_cb(status, result):
    print ''
    print 'Result'
    print '------'
    if result:
        for i, p, v in zip(range(1, len(poses) + 1), goal.path.poses,
                           result.visited):
            print '%2i) %s %s' % (i, 'visited' if v else 'skipped',
                                  to_str(p.pose))
    print ''
    state = ['pending', 'active', 'preempted', 'succeeded', 'aborted',
             'rejected', 'preempting', 'recalling', 'recalled',
             'lost'][status]
    print 'Action completed, state:', state
    rospy.signal_shutdown('Path execution completed')


if __name__ == '__main__':
    rospy.init_node(NODE, anonymous=True)

    SERVER = '/path_executor/execute_path'
    ep_client = SimpleActionClient(SERVER, ExecutePathAction)
    print 'Connecting to [%s] server...' % SERVER
    ep_client.wait_for_server()

    poses = list()
    goal = ExecutePathGoal()
    inputs = np.asarray([[-5,-5,0],[-6,-6,1.57],[-7,-7,0]])
    for index in xrange(len(inputs[:,0])):
        
        x = inputs[index,0]
        y = inputs[index,1]
        yaw = inputs[index,2]

        p = PoseStamped()
        q = quaternion_from_euler(0, 0, yaw)
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        goal.path.poses.append(p)
        poses.append((x, y, yaw))
    goal.skip_unreachable = True
    rospy.sleep(0.2)
    print ''
    print 'Goal'
    print '----'
    print 'Poses:'
    for i, p in enumerate(goal.path.poses):
        print '%2i) %s' % (i + 1, to_str(p.pose))

    ep_client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)
    print ''
    print 'Feedback'
    print '--------'
    rospy.spin()


   
    