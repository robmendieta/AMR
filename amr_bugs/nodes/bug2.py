#!/usr/bin/env python

PACKAGE = 'amr_bugs'
NODE = 'bug2'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import tf
import smach
from smach_ros import SimpleActionState, ActionServerWrapper
import dynamic_reconfigure.client
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped

from amr_msgs.msg import MoveToAction, MoveToActionGoal
from amr_bugs.bug_brain import BugBrain
from amr_bugs.bug_brain_visualizer import BugBrainVisualizer


class NewGoalState(smach.State):
    """
    This state retrieves the wallfollower mode from its dynamic reconfiguration
    server and creates a new instance of BugBrain for the new goal.
    """
    def __init__(self, ws):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['action_goal'])
        self.ws = ws
        self.d_client = dynamic_reconfigure.client.Client('wallfollower')

    def execute(self, ud):
        wallfollower_config = self.d_client.get_configuration()
        self.ws.new_brain(ud.action_goal.target_pose.pose.position.x,
                          ud.action_goal.target_pose.pose.position.y,
                          wallfollower_config['mode'])
        return 'succeeded'


class WallfollowerState(smach.State):
    """
    This state enables the wallfollower node and goes into an infinite loop
    until BugBrain decides that it is time to leave the wall, or until a
    preemption request is received. Then it disables the wallfollower node and
    returns.
    """
    def __init__(self, freq=10):
        smach.State.__init__(self, outcomes=['leave_wall',
                                             'unreachable_goal',
                                             'preempted',
                                             'aborted'])
        self.tf_listener = tf.TransformListener(True, rospy.Duration(5))
        self.brain = None
        self.brain_vis = None
        self.frequency = freq

    def execute(self, ud):
        # Try to enable wallfollower
        if not self.enable_wallfollower():
            return 'aborted'
        # Notify brain that we started to follow a wall
        self.brain.follow_wall(*self.get_current_pose())
        # Go into infinite loop until preemped or shut down
        r = rospy.Rate(self.frequency)
        while not rospy.is_shutdown() and not self.preempt_requested():
            x, y, theta = self.get_current_pose()
            # Check brain's belief about the reachability of the goal
            if self.brain.is_goal_unreachable(x, y, theta):
                self.disable_wallfollower()
                return 'unreachable_goal'
            # Check if it is the time to leave the wall
            if self.brain.is_time_to_leave_wall(x, y, theta):
                # Try to disable wallfollower
                if not self.disable_wallfollower():
                    return 'aborted'
                # Notify brain that we left the wall
                self.brain.leave_wall(x, y, theta)
                return 'leave_wall'
            r.sleep()
        # In case of preemption also disable wallfollower
        self.disable_wallfollower()
        self.service_preempt()
        return 'preempted'

    def new_brain(self, x, y, mode):
        """Create a new instance of BugBrain (and visualizer as well)."""
        if self.brain_vis:
            self.brain_vis.shutdown()
        self.brain = BugBrain(x, y, mode)
        self.brain_vis = BugBrainVisualizer(self.brain)

    def get_current_pose(self):
        """Get current (2D) pose as a tuple (x, y, theta)."""
        time = self.tf_listener.getLatestCommonTime('odom', 'base_link')
        p, q = self.tf_listener.lookupTransform('odom', 'base_link', time)
        e = tf.transformations.euler_from_quaternion(q)
        return p[0], p[1], e[2]

    def enable_wallfollower(self):
        try:
            rospy.ServiceProxy('wallfollower/enable', Empty)()
            return True
        except rospy.ServiceException:
            return False

    def disable_wallfollower(self):
        try:
            rospy.ServiceProxy('wallfollower/disable', Empty)()
            return True
        except rospy.ServiceException:
            return False

if __name__ == '__main__':
    rospy.init_node(NODE)

    sm = smach.StateMachine(input_keys=['action_goal'],
                            outcomes=['succeeded',
                                      'preempted',
                                      'aborted',
                                      'unreachable_goal'])
    sm.userdata.action_feedback = None
    ws = WallfollowerState()
    with sm:
        smach.StateMachine.add('NEW_GOAL',
                               NewGoalState(ws),
                               transitions={'succeeded': 'MOVE_TO'})
        smach.StateMachine.add('MOVE_TO',
                               SimpleActionState('/motion_controller/move_to',
                                                 MoveToAction,
                                                 goal_key='action_goal'),
                               transitions={'preempted': 'aborted',
                                            'aborted': 'WALLFOLLOWER'})
        smach.StateMachine.add('WALLFOLLOWER',
                               ws,
                               transitions={'leave_wall': 'MOVE_TO'})

    # Wrap an action server around the state machine.
    asw = ActionServerWrapper('~move_to',
                              MoveToAction,
                              wrapped_container=sm,
                              succeeded_outcomes=['succeeded'],
                              aborted_outcomes=['aborted, unreachable_goal'],
                              preempted_outcomes=['preempted'])

    # Similarly to how it is done in MotionController, create a combination of
    # listened and re-publisher to allow the user to send a goal as a message
    # (not as an action service request).
    action_goal_pub = rospy.Publisher('~move_to/goal', MoveToActionGoal)

    def simple_goal_cb(target_pose):
        rospy.loginfo('Received target pose through the "simple goal" topic. '
                      'Wrapping it in the action message and forwarding to '
                      'the server.')
        msg = MoveToActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.goal.target_pose = target_pose
        action_goal_pub.publish(msg)

    simple_goal_sub = rospy.Subscriber('~move_to_simple/goal', PoseStamped,
                                       simple_goal_cb)

    # Run the server
    asw.run_server()
    rospy.spin()
