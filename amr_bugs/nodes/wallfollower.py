#!/usr/bin/env python

PACKAGE = 'amr_bugs'
NODE = 'wallfollower'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import smach_ros
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse

from amr_msgs.msg import Ranges
from amr_srvs.srv import SwitchRanger
from amr_bugs.cfg import WallfollowerConfig
from amr_bugs import wallfollower_state_machine


class WallfollowerNode:
    def __init__(self):
        rospy.init_node(NODE)
        # Wait until SwitchRanger service becomes available
        rospy.loginfo('Waiting for the /switch_ranger service...')
        rospy.wait_for_service('/switch_ranger')
        try:
            # Make sure that the pioneer sonars are available and enable them
            rospy.ServiceProxy('/switch_ranger', SwitchRanger)('sonar_pioneer', True)
            rospy.loginfo('Enabled pioneer sonars.')
        except rospy.ServiceException:
            rospy.logerr('Pioneer sonars are not available, shutting down.')
            exit()
        # Create state machine
        self.sm = wallfollower_state_machine.construct()
        # Create a sonar subscriber
        self.sonar_sub = rospy.Subscriber('/sonar_pioneer', Ranges, self.sonar_cb)
        # Create a velocity publisher
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist)
        # Create a dynamic reconfigure server
        self.reconfigure_srv = Server(WallfollowerConfig, self.reconfigure_cb)
        # Create an introspection server for visual inspection of the machine
        self.sis = smach_ros.IntrospectionServer('smach_inspector', self.sm, '/WALLFOLLOWER')
        self.sis.start()
        # Create start and stop services
        self.enable_srv = rospy.Service('~enable', Empty, self.enable_cb)
        self.disable_srv = rospy.Service('~disable', Empty, self.disable_cb)
        # Check parameter to know whether to enable itself on startup
        if rospy.get_param('~enable_at_startup', True):
            self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
            rospy.Timer(rospy.Duration(0.5), lambda e: self.sm.execute(), oneshot=True)

    def sonar_cb(self, msg):
        self.sm.set_ranges(msg.ranges)

    def timer_cb(self, event):
        self.velocity_pub.publish(self.sm.get_twist())

    def reconfigure_cb(self, config, level):
        return self.sm.set_config(config)

    def enable_cb(self, request):
        rospy.loginfo('Received [enable] request...')
        # Create a timer to periodically trigger velocity publishing
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        # Execute the state machine
        # The StateMachine.execute() function blocks, but we need to return
        # from this callback immediately. Therefore this trick with a one shot
        # timer. Effectively this will just call execute in another thread.
        rospy.Timer(rospy.Duration(0.1), lambda e: self.sm.execute(), oneshot=True)
        return EmptyResponse()

    def disable_cb(self, request):
        rospy.loginfo('Received [disable] request...')
        # Stop publishing velocities (we are no longer in control of the robot)
        # Timer object may not exist if stop was called before start
        try:
            self.timer.shutdown()
        except AttributeError:
            pass
        self.velocity_pub.publish(Twist())
        # Stop the state machine
        self.sm.request_preempt()
        return EmptyResponse()

if __name__ == '__main__':
    w = WallfollowerNode()
    rospy.spin()
