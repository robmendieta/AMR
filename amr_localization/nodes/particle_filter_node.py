#!/usr/bin/env python

PACKAGE = 'amr_localization'
NODE = 'particle_filter'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import tf
import numpy as np
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from amr_srvs.srv import GetMultiplePoseLikelihood, GetMultiplePoseLikelihoodRequest
from amr_localization.motion_model import MotionModel
from amr_localization.particle_filter import ParticleFilter
from amr_localization.pose import Pose
from geometry_msgs.msg import PoseStamped
from amr_localization.particle import Particle
from amr_localization.particle_visualizer import ParticleVisualizer

class ParticleFilterNode:
    """
    This is a port of the AMR Python ParticleFilterNode
    """
    def __init__(self):
        
        rospy.init_node(NODE)

        """
            Private variables
        """
        self.beam_transforms_ = []
        self.previous_pose_ = np.identity(4)

        """
            Parameters
        """
        self.update_rate = rospy.get_param('update_rate', 5.0)
        self.frame_id = rospy.get_param('frame_id', "pf_pose")
        # This will block until the stage node is loaded and has set the world dimensions parameters
        self.world_width = 0
        self.world_height = 0
        rospy.loginfo('getting params')
        while not rospy.has_param('/world_width') or not rospy.has_param('/world_height'):
            time.sleep(0.01)
        self.world_width = rospy.get_param('/world_width')
        self.world_height = rospy.get_param('/world_height')

        # Compute world extent
        self.min_x = - self.world_width / 2.0
        self.max_x = self.world_width / 2.0
        self.min_y = - self.world_height / 2.0
        self.max_y = self.world_height / 2.0

        """
            Services
        """
        self.pose_likelihood_client_ = rospy.ServiceProxy('pose_likelihood_server/get_pose_likelihood', GetMultiplePoseLikelihood)

        """
            Topics
        """
        self._tf= tf.TransformListener()
        self._tfbr = tf.TransformBroadcaster()
        #self._simple_goal_subscriber = rospy.Subscriber('pose_estimate',
        #                                                PoseWithCovarianceStamped,
        #                                                self.poseEstimateCallback,
        #                                                queue_size=1)

        # Create particle filter and visualizer
        self.particle_filter = ParticleFilter(self.min_x, self.max_x, self.min_y, self.max_y, self.computeParticlesWeights)
        self.particle_visualizer = ParticleVisualizer("particles", "odom")

        # Schedule periodic filter updates
        self.update_timer_ = rospy.Timer(rospy.Duration(1/self.update_rate), self.updateCallback)
        self.previous_pose_ = np.identity(4)
        rospy.loginfo('Started [particle_filter] node.')

    def updateCallback(self, event):
        # Determine the motion since the last update
        #tf::StampedTransform transform;
        try:
            now = rospy.Time.now()
            #TODO wait for transform
            transform = self._tf.lookupTransform('base_link', 'odom', now)
            transform = tf.TransformerROS().fromTranslationRotation(transform[0],transform[1])
        except tf.Exception, e:
            rospy.logerr('Service call failed: %s'%e)
            return

        delta_transform = np.dot(self.previous_pose_, tf.transformations.inverse_matrix(transform))
        self.previous_pose_ = transform

        # In theory the obtained transform should be imprecise (because it is
        # based on the wheel odometry). In our system, however, we in fact get the
        # ground truth transform. It is therefore impossible to simulate the
        # "kidnapped robot" problem, because even if the robot is teleported in a
        # random spot in the world, the transform will be exact, and the particle
        # filter will not lose the track.
        # We test the length of the transform vector and if it exceeds some "sane"
        # limit we assume that the odometry system "failed" and feed identity
        # transform to the particle filter.
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(delta_transform)

        if len(trans) > 2.0:
            delta_transform = np.identity(4)

        # Perform particle filter update, note that the transform is in robot's
        # coordinate frame
        forward = trans[0]
        lateral = trans[1]
        yaw = angles[2]
        self.particle_filter.update(forward,lateral,yaw)

        # Visualize the particle set, broadcast transform, and print some information
        particles = self.particle_filter.get_particles()
        avg_weight = 0.0
        for p in particles:
            avg_weight += p.weight
        avg_weight = avg_weight / len(particles)

        self.particle_visualizer.publish(particles);

        self.broadcastTransform()

        rospy.loginfo('Motion: [' + str(forward) + ',' + str(lateral) + ',' + str(yaw) + '] Average particle weight: ' + str(avg_weight))

      # Broadcast the current estimation of robot's position.
    def broadcastTransform(self):
        estimated_pose = self.particle_filter.get_pose_estimate()
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, estimated_pose.theta)
        self._tfbr.sendTransform((estimated_pose.x, estimated_pose.y, 2.0), quat, rospy.Time.now(), self.frame_id, "odom")
        pass


    # Compute the weight of a particle.
    # This function takes the pose from the particle, and uses external service
    # to determine how likely is that the robot is in this pose given the data
    # that it sensed.
    def computeParticlesWeights(self, particles):
        request = GetMultiplePoseLikelihoodRequest()
        for p in particles:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = rospy.Time.now()
            point = Point(p.pose.x, p.pose.y, 0.0)
            pose.pose.position = point
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, p.pose.theta)
            pose.pose.orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
            request.poses.append(pose)

        #TODO safe service request
        response = self.pose_likelihood_client_(request)
        return response.likelihoods

    # This callback is triggered when someone sends a message with a pose
    # estimate to the "~/pose_estimate" topic.
    #def poseEstimateCallback(self, pose_estimate):
    #    rospy.loginfo('pose estimation cb called')
    #    p = Pose()
    #    p.x = pose_estimate.pose.pose.position.x
    #    p.y = pose_estimate.pose.pose.position.y
    #    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(pose_estimate.pose.pose.orientation)
    #    p.theta = yaw
    #    rospy.loginfo("Received pose estimate " + str(p) + ", forwarding it to the ParticleFilter object")
    #    self.particle_filter.set
    #    #TODO particle_filter_->setExternalPoseEstimate(p);
    #    pass

if __name__ == '__main__':
    rospy.loginfo('Trying to start ParticleFilterNode')
    w = ParticleFilterNode()

    rospy.spin()

