#!/usr/bin/env python

PACKAGE = 'amr_braitenberg'
NODE = 'braitenberg_vehicle'

import rospy

from amr_srvs.srv import SwitchRanger, SwitchRangerRequest
from amr_msgs.msg import WheelSpeeds, Ranges
from amr_braitenberg.braitenberg_vehicle import BraitenbergVehicle
import amr_braitenberg.cfg.BraitenbergVehicleConfig as BraitenbergVehicleConfig

import dynamic_reconfigure.server


class BraitenbergVehicleNode:
    """
    BraitenbergVehicle node ported and refactored.
    """
    
    
    def __init__(self):
        
        rospy.init_node(NODE, log_level=rospy.DEBUG)
        
        # Wait until SwitchRanger service (and hence stage node) becomes available:
        rospy.loginfo('Waiting for the /switch_ranger service to be advertised...')
        switch_ranger_client = rospy.ServiceProxy('/switch_ranger', SwitchRanger)
        
        switch_ranger_client.wait_for_service()
        # Make sure that the braitenberg sonars are available and enable them.
        srr = SwitchRangerRequest()
        srr.name = 'sonar_braitenberg'
        srr.state = True
        
        if switch_ranger_client.call(srr):
            rospy.loginfo('Enabled braitenberg sonars.')
        else:
            rospy.logerr('Braitenberg sonars are not available, shutting down.')
            exit()
        self._vehicle = BraitenbergVehicle()
        
        """
            Subscribers and publishers
        """
        
        self._sonar_subscriber = rospy.Subscriber('/sonar_braitenberg',
                                                  Ranges,
                                                  self._sonar_callback,
                                                  queue_size=100)
        self._wheel_speeds_publisher = rospy.Publisher('/cmd_vel_diff',
                                                       WheelSpeeds,
                                                       queue_size=100)
        self._dynamic_reconfigure = dynamic_reconfigure.server.Server(
                                                BraitenbergVehicleConfig,
                                                self._reconfigure_callback)
        rospy.loginfo('Started [braitenberg_vehicle] node.')
    
    
    def _sonar_callback(self, ranges_msg):
        """
        ========================= YOUR CODE HERE =========================
        This node subscribes to the Braitenberg vehicle sonars.
        This function is called every time the Ranges message arrives.
        See the declaration of the ranges message in amr_msgs/msg/Ranges.msg
        for the message contents.

        Instructions: based on the ranges reported by the two
                      sonars compute the wheel speds and fill
                      in the WheelSpeeds message.
                      
                      Publish the message using
                      self._wheel_speeds_publisher


        Hint: use self._vehicle.compute_wheel_speeds(...) function
        ==================================================================
        """
        ws = WheelSpeeds()


        # Output the debug info:
        rospy.logdebug('[{:.2f}, {:.2f}] --> [{:.2f}, {:.2f}]'.format(
                                                      ranges_msg.ranges[0].range,
                                                      ranges_msg.ranges[1].range,
                                                      ws.speeds[0],
                                                      ws.speeds[1]))
    
    
    def _reconfigure_callback(self, config, params):
        """
        ========================= YOUR CODE HERE =========================
        This function is called on the reconfiguration request via dynamic reconfigure.
        The dynamic reconfigurable parameters are declared in
        amr_braitenberg/cfg/BraitenbergVehicle.cfg

        Instructions: pass the new parameters to your BraitenbergVehicle instance
                      self._vehicle.set_params(....)

        Hint: see the logdebug message below for an example how to access config parameters.
        ==================================================================
        """


        rospy.logdebug('Vehicle reconfigured: type {}, '
                       'factors {:.2f}] and {:.2f}]'.format(
                                                           ['A','B','C'][config.type],
                                                           config.factor1,
                                                           config.factor2))
        return config


if __name__ == '__main__':
    n = BraitenbergVehicleNode()
    rospy.spin()
