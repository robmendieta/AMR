#!/usr/bin/env python

PACKAGE = 'amr_localization'
NODE = 'pose_likelihood_server'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import tf
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose2D
from amr_srvs.srv import GetMultiplePoseLikelihood, GetMultiplePoseLikelihoodResponse, GetNearestOccupiedPointOnBeam, GetNearestOccupiedPointOnBeamRequest, SwitchRanger


class PoseLikelihoodServerNode:
    """
    This is a port of the AMR Python PoseLikelihoodServerNode
    """

    def __init__(self):
        """Variables definition"""
        self.angle_upper = 0.0
        self.angle_lower= 0.0
        self.angle_step=0.0
        self.number_of_beams=0
        self.real_observations = []
        self.likelihood = []


        rospy.init_node(NODE)

        # Wait until SwitchRanger service (and hence stage node) becomes available.
        rospy.loginfo('Waiting for the /switch_ranger service to be advertised...');
        rospy.wait_for_service('/switch_ranger')
        try:
            switch_ranger = rospy.ServiceProxy('/switch_ranger', SwitchRanger)
            # Make sure that the hokuyo laser is available and enable them (aka switch on range scanner)
            switch_ranger('scan_front', True)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)

        """
            Expose GetMultiplePoseLikelihood Service here,
            subscribe for /scan_front,
            create client for /occupancy_query_server/get_nearest_occupied_point_on_beam service

            http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        """

        self._tf = tf.TransformListener()
        """ Subscriber for getting the laser readings and the required information for the beams."""
        self._laser_subscriber = rospy.Subscriber('/scan_front',LaserScan,self.laser_callback,queue_size=50)
        """Advertising of the service of likelihood."""
        s = rospy.Service('/pose_likelihood_server/get_pose_likelihood',GetMultiplePoseLikelihood,self.likelihood_callback)
                
        """Initialization of the service GetNearestOccupiedPointOnBeam."""
        rospy.wait_for_service('/occupancy_query_server/get_nearest_occupied_point_on_beam')
        try:
            self.occupied_points_client = rospy.ServiceProxy('/occupancy_query_server/get_nearest_occupied_point_on_beam', GetNearestOccupiedPointOnBeam)
            #occupied_points_client('pose',self.likelihood)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)

        rospy.loginfo('Started [pose_likelihood_server] node.')

        #pass
    """Callback function: laser_callback
    Function to get the readings of the laser. The angle_upper, angle_lower and angle_step parameters are used to compute the orientation of each beam."""
    def laser_callback(self,data):
        self.real_observations = []
        self.angle_upper = data.angle_max
        self.angle_lower = data.angle_min
        self.angle_step = data.angle_increment
        self.range_max = data.range_max
        self.number_of_beams = len(data.ranges)

        for i in xrange(self.number_of_beams):
            self.real_observations.append(data.ranges[i])



    def likelihood_callback(self, response):

        #storing
        multiposes = response.poses
        #To store the probabilities
        likelihood_array=[]

        for member in xrange(len(multiposes)):
    
            #To calculate the probability according the formula for distribution in slides
            sigma = 0.4
            missmatches_counter = 0
            beam_weight = 0.0
            weight_sum = 0.0
            #creating the service request
            occupied_points_request = GetNearestOccupiedPointOnBeamRequest()
            #request.beams.append(pose)
            pose = self.get_beam_pose(multiposes[member])
            occupied_points_request.beams = pose
            occupied_points_request.threshold = 2
            beamer_iterator = 0                
            distance_prediction = 0.0
            #request to client to get distances
            occupied_points_client_request= self.occupied_points_client(occupied_points_request)
            
            for beamer_iterator in xrange(self.number_of_beams): 
                distance_prediction = occupied_points_client_request.distances[beamer_iterator]
                real_distance = self.real_observations[beamer_iterator]
                euclidean_distance = abs(distance_prediction - real_distance)
                
                #Clamping for min/max values of the distance
                if(distance_prediction < 0.0):
                    distance_prediction = 0.0
                elif(distance_prediction > self.range_max):
                    distance_prediction = self.range_max


                if(euclidean_distance <= 2*sigma):
                    #Probability distribution: Determine likelihood for measured distance
                    beam_weight = (1.0 / (sigma*math.sqrt(2*math.pi))) * math.exp((-math.pow(distance_prediction - real_distance, 2.0)) / (2 * math.pow(sigma, 2.0)))
                    weight_sum += beam_weight
                    #Up to 4 missmatches accepted
                elif euclidean_distance > 2*sigma :
                    missmatches_counter = missmatches_counter+1

                
            if(missmatches_counter >= 4):
               weight_sum = 0
            else:
               weight_sum = weight_sum /self.number_of_beams
               
            likelihood_array.append(weight_sum)
        
        multipose_response = GetMultiplePoseLikelihoodResponse(likelihood_array)

        return multipose_response
        


    """Transform a point from the frame LaserFront to the frame of the Robot
    - base_link: frame of the Robot.
    - base_laser_front_link: frame of the LaserFront."""

    def get_beam_pose(self, robot_pose):
        twelve_beam_poses=[]
        #Transform lasers to robot frame
        try:
            time = self._tf.getLatestCommonTime("/base_link","/base_laser_front_link")
            position, quaternion = self._tf.lookupTransform("/base_link","/base_laser_front_link",time)
            yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
            x, y, yaw = position[0], position[1], yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerror("Error tf")

        #calling the service
        # Getting the 12 poses of laser beams in the robot frame.
        for i in range(self.number_of_beams):
            #Data type of the returned msg. Poses with x, y and theta.
            beam_pose = Pose2D()
            local_beam_orientation = (self.angle_lower + (i*self.angle_step))
            #Converting negatives angles to its positive equivalence.
            if (local_beam_orientation < 0):
                local_beam_orientation = local_beam_orientation + 2*math.pi
            #Creating the frame of each laser. This frames will be only rotated relative to the robot's frame.
            orientations = (robot_pose.pose.orientation.x,
                            robot_pose.pose.orientation.y,
                            robot_pose.pose.orientation.z,
                            robot_pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(orientations)
            beam_pose.x = robot_pose.pose.position.x + x
            beam_pose.y = robot_pose.pose.position.y + y
            beam_pose.theta = local_beam_orientation + euler[2] + yaw
            twelve_beam_poses.append(beam_pose)

        return twelve_beam_poses


    """
    ============================== YOUR CODE HERE ==============================
    Instructions:   implemenent the pose likelihood server node including a
                    constructor which should create all needed servers, clients,
                    and subscribers, and appropriate callback functions.
                    GetNearestOccupiedPointOnBeam service allows to query
                    multiple beams in one service request. Use this feature to
                    simulate all the laser beams with one service call, otherwise
                    the time spent on communication with the server will be too
                    long.

    Hint: refer to the sources of the previous assignments or to the ROS
          tutorials to see examples of how to create servers, clients, and
          subscribers.

    Hint: in the laser callback it is enough to just store the incoming laser
          readings in a class member variable so that they could be accessed
          later while processing a service request.

    Hint: the GetNearestOccupiedPointOnBeam service may return arbitrary large
          distance, do not forget to clamp it to [0..max_range] interval.


    Look at the tf library capabilities, you might need it to find transform
    from the /base_link to /base_laser_front_link.
    Here's an example how to use the transform lookup:

        time = self._tf.getLatestCommonTime(frame_id, other_frame_id)
        position, quaternion = self._tf.lookupTransform(frame_id,
                                                        other_frame_id,
                                                        time)
        yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        x, y, yaw = position[0], position[1], yaw

    You might need other functions for transforming routine, you can find
    a brief api description
    http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/python/tf_python.html
    """


if __name__ == '__main__':
    w = PoseLikelihoodServerNode()
    rospy.spin()
