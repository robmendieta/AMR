#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'motion_controller'

import rospy
import tf
import math

from actionlib.simple_action_server import SimpleActionServer
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from amr_msgs.msg import MoveToAction, MoveToActionGoal, MoveToResult ,Obstacle

from amr_navigation.velocity_controller import Velocity
from amr_navigation.diff_velocity_controller import DiffVelocityController
from amr_navigation.omni_velocity_controller import OmniVelocityController


class MotionControllerNode:
    """
    This is a port of the AMR C++ MotionControllerNode
    """

    CONTROLLER_TYPE_DIFF = 'diff'
    CONTROLLER_TYPE_OMNI = 'omni'
    CONTROLLER_TYPE_UNSPECIFIED = 'unsp'


    def __init__(self):
        
        rospy.init_node(NODE)
        
        """
            Parameters
        """
        max_linear_velocity = rospy.get_param('max_linear_velocity', 0.3)
        max_linear_acceleration = rospy.get_param('max_linear_acceleration', 0.05)
        linear_tolerance = rospy.get_param('linear_tolerance', 0.02)
        max_angular_velocity = rospy.get_param('max_angular_velocity', 0.2)
        max_angular_acceleration = rospy.get_param('max_angular_acceleration', 0.03)
        angular_tolerance = rospy.get_param('angular_tolerance', 0.02)
        
        abort_if_obstacle_detected = rospy.get_param('abort_if_obstacle_detected', True)
        self._controller_frequency = rospy.get_param('controller_frequency', 10.0)
        
        controller_type = rospy.get_param('~controller', self.CONTROLLER_TYPE_UNSPECIFIED)
        
        if controller_type == self.CONTROLLER_TYPE_DIFF:
            #Create diff controller
            self._velocity_controller = DiffVelocityController(max_linear_velocity,
                                                               linear_tolerance,
                                                               max_angular_velocity,
                                                               angular_tolerance)
        elif controller_type == self.CONTROLLER_TYPE_OMNI:
            #Create omni controller
            """
            ========================= YOUR CODE HERE =========================

            Instructions: create an instance of OmniVelocityController.     
            Hint: you may copy-paste from the DiffVelocityController case
            and adjust the arguments in the call to the constructor to
            conform to what you have implemented in that class.
            
            """
        elif controller_type == self.CONTROLLER_TYPE_UNSPECIFIED:
            rospy.logerr('Controller type not specified. '
                         'Check the [controller] launch parameter')
            exit()
        else:
            #Unknown controller
            rospy.logerr('Requested controller type "{0}" unknown. '
                         'Check the [controller] launch parameter'.format(controller_type))
            exit()
        
        """
            Publishers
        """
        self._velocity_publisher = rospy.Publisher('/cmd_vel', Twist,
                                                   queue_size=10)
        self._current_goal_publisher = rospy.Publisher(NODE+'/current_goal',
                                                       PoseStamped,
                                                       latch=True,
                                                       queue_size=0)
        self._action_goal_publisher = rospy.Publisher(NODE+'/move_to/goal',
                                                      MoveToActionGoal,
                                                      queue_size=1)
        
        """
            Subscribers
        """
        self._simple_goal_subscriber = rospy.Subscriber(NODE+'/move_to_simple/goal',
                                                        PoseStamped,
                                                        self._simple_goal_callback,
                                                        queue_size=1)
        if abort_if_obstacle_detected:
            self._obstacles_subscriber = rospy.Subscriber('obstacles',
                                                          Obstacle,
                                                          self._obstacles_callback,
                                                          queue_size=100)
        """
            Action server
        """
        self._move_to_server = SimpleActionServer(NODE+'/move_to',
                                                  MoveToAction,
                                                  self._move_to_callback,
                                                  auto_start=False)
        self._move_to_server.start()
        self._tf= tf.TransformListener()
        rospy.loginfo('Started [motion_controller] node.')


    def _move_to_callback(self, move_to_goal):
        """
        Triggered with a request to move_to action server
        """
        rospy.loginfo('Received [move_to] action command.')
        if not self._set_new_goal(move_to_goal):
            return
        else:
            rate = rospy.Rate(self._controller_frequency)
            while not rospy.is_shutdown():
                if not self._move_to_server.is_active():
                    # Exit if the goal was aborted
                    return
                if self._move_to_server.is_preempt_requested():
                    # Process pending preemption requests
                    rospy.loginfo('Action preemption requested.')
                    if (    self._move_to_server.is_new_goal_available() and
                            self._set_new_goal(self._move_to_server.accept_new_goal())
                       ):
                        # New goal already set
                        pass
                    else:
                        # No new goals, preempt explicitly and exit the callback
                        self._publish_zero_velocity()
                        self._move_to_server.set_preempted()
                        return
                if not self._move_towards_goal():
                    #  Finish execution if the goal was reached
                    self._move_to_server.set_succeeded(MoveToResult(), 'Goal reached.')
                    self._publish_zero_velocity()
                    return
                rate.sleep()
            self._move_to_server.set_aborted(MoveToResult(), 'Aborted. The node has been killed.')


    def _simple_goal_callback(self, target_pose):
        """
        Wrapper for simple goal action. Forwards as a request to the move_to
        action server. Has to be tested!
        """
        rospy.loginfo('Received target pose through the "simple goal" topic.'
                      'Wrapping it in the action message and forwarding to the server.')
        rospy.logwarn('Simple goal control is yet to be tested!')
        action_goal = MoveToActionGoal()
        action_goal.header.stamp = rospy.Time.now()
        action_goal.goal.target_pose = target_pose
        self._action_goal_publisher.publish(action_goal)


    def _obstacles_callback(self, obstacle_msg):
        rospy.logwarn('An obstacle was detected. Will stop the robot and cancel the current action.')
        if self._move_to_server.is_active():
            self._move_to_server.set_aborted(MoveToResult(), 'Obstacle encountered, aborting...')
        self._publish_zero_velocity()


    def _move_towards_goal(self):
        try:
            time = self._tf.getLatestCommonTime("odom", "base_footprint")
            position, quaternion = self._tf.lookupTransform("odom", "base_footprint", time)
        except Exception as ex:
            rospy.logwarn('Transform lookup failed (\odom -> \base_footprint). '
                          'Reason: {0}.'.format(ex.message))
            return True
        current_pose = Pose2D()
        current_pose.x, current_pose.y = position[0], position[1]
        current_pose.theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        velocity = self._velocity_controller.compute_velocity(current_pose)
        
        if self._velocity_controller.is_target_reached():
            rospy.loginfo('The goal was reached')
            return False
        else:
            self._publish_velocity(velocity)
            return True


    def _set_new_goal(self, new_goal):
        """
        Set new target pose as given in the goal message.
        Checks if the orientation provided in the target pose is valid.
        Publishes the goal pose for the visualization purposes.
        Returns true if the goal was accepted.
        """
        if not self._is_quaternion_valid(new_goal.target_pose.pose.orientation):
            rospy.logwarn('Aborted. Target pose has invalid quaternion.')
            self._move_to_server.set_aborted(MoveToResult(),'Aborted. Target pose has invalid quaternion.')
            return False
        else:
            x = new_goal.target_pose.pose.position.x
            y = new_goal.target_pose.pose.position.y
            yaw = tf.transformations.euler_from_quaternion([new_goal.target_pose.pose.orientation.x,
                                                            new_goal.target_pose.pose.orientation.y,
                                                            new_goal.target_pose.pose.orientation.z,
                                                            new_goal.target_pose.pose.orientation.w])[2]
            pose = Pose2D(x, y, yaw)
            self._velocity_controller.set_target_pose(pose)
            self._current_goal_publisher.publish(new_goal.target_pose)
            rospy.loginfo('New target pose: {0}'.format(pose))
            return True


    def _publish_zero_velocity(self):
        self._publish_velocity(Velocity())


    def _publish_velocity(self, vel):
        self._velocity_publisher.publish(vel.get_twist())


    def _is_quaternion_valid(self, q):
        if any([math.isinf(a) or math.isnan(a) for a in [q.x, q.y, q.z, q.w]]):
            rospy.logwarn('Quaternion has NaN\'s or infinities.')
            return False
        # TODO: check quaternion length and rotation test
        return True
        pass


if __name__ == '__main__':
    w = MotionControllerNode()
    rospy.spin()

