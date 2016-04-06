#!/usr/bin/env python

PACKAGE = 'amr_braitenberg'
NODE = 'differential_drive_emulator'

import rospy
from geometry_msgs.msg import Twist
from amr_msgs.msg import WheelSpeeds


class DifferentialDriveEmulatorNode:
    
    def __init__(self):
        rospy.init_node(NODE)
        # read differential drive params:
        self._wheel_diameter = rospy.get_param('wheel_diameter', 0.15)
        self._distance_between_wheels = rospy.get_param('distance_between_wheels', 0.5)
        
        self._wheel_speed_subscriber = rospy.Subscriber('/cmd_vel_diff',
                                                        WheelSpeeds,
                                                        self._wheel_speed_callback,
                                                        queue_size=100)
        self._velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.loginfo('Started differential drive emulator node.')


    def _wheel_speed_callback(self, msg):
        if len(msg.speeds)!=2:
            rospy.logwarn('Ignoring WheelSpeeds message because it does not '
                          'contain two wheel speeds.')
            return
        else:
            twist = Twist()
            """
            ==================== YOUR CODE HERE ====================
            Instructions: compute linear and angular components based
                          on robot geometrical parameters and
                          fill in the twist message.

            You may lookup the geometry_msgs.msg declaration at ros.org
            ========================================================
            """

            self._velocity_publisher.publish(twist)
            rospy.logdebug('[{:.2f} {:.2f}] --> [{:.2f} {:.2f}]'.format(msg.speeds[0],
                                                                        msg.speeds[1],
                                                                        twist.linear.x,
                                                                        twist.angular.z))


if __name__ == '__main__':
    n = DifferentialDriveEmulatorNode()
    rospy.spin()
