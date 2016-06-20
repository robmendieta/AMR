#!/usr/bin/env python

PACKAGE = 'amr_localization'
NODE = 'particle_visualizer'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import tf
import math
from amr_localization.motion_model import MotionModel
from amr_localization.pose import Pose
from amr_localization.particle import Particle
from amr_localization.random_particle_generator import RandomParticleGenerator
from visualization_msgs.msg import Marker, MarkerArray

class ParticleVisualizer:

    def __init__(self, topic_name, frame_id):
        self.topic_name = topic_name
        self.frame_id = frame_id
        self.marker_publisher = rospy.Publisher(topic_name, MarkerArray)

    def publish(self, particles):
        min_val = min(particles).weight
        max_val = max(particles).weight
        range = (max_val-min_val)*0.9
        markers = MarkerArray()
        for p in particles:
            marker = Marker()
            marker.ns = "particles"
            marker.header.frame_id = self.frame_id;
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.6;
            marker.scale.y = 0.1;
            marker.scale.z = 0.2;
            if range > 0:
                marker.color.a = 0.1 + (p.weight - min_val) / range
            else:
                marker.color.a = 1.0
            marker.color.r = 0.0;
            marker.color.g = 0.2;
            marker.color.b = 0.8;
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, p.pose.theta)
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            marker.pose.position.x = p.pose.x;
            marker.pose.position.y = p.pose.y;
            marker.pose.position.z = 0.05;
            marker.id = len(markers.markers);
            markers.markers.append(marker);
        self.marker_publisher.publish(markers)