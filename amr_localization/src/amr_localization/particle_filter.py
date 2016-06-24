#!/usr/bin/env python

PACKAGE = 'amr_localization'
NODE = 'particle_filter'

import roslib
import numpy as np
roslib.load_manifest(PACKAGE)
import rospy
import math
import random
from amr_localization.motion_model import MotionModel
from amr_localization.pose import Pose
from amr_localization.particle import Particle
from amr_localization.random_particle_generator import RandomParticleGenerator

class ParticleFilter:

    def __init__(self, map_min_x, map_max_x, map_min_y, map_max_y, weigh_particles_callback):
        self.weigh_particles_callback = weigh_particles_callback
        self.particle_set_size = 100
        self.random_particles_size = 20
        self.motion_model = MotionModel(0.02, 0.01)
        self.random_particle_generator = RandomParticleGenerator(map_min_x, map_max_x, map_min_y, map_max_y)
    
        self.particles = []
        for i in range(self.particle_set_size):
            self.particles.append(self.random_particle_generator.generate_particle())
        
        self.pose_estimate = Pose()

    def update(self, x, y, yaw):
        '''
        ============================== YOUR CODE HERE ==============================
         Instructions: do one complete update of the particle filter. It should
                       generate new particle set based on the given motion
                       parameters and the old particle set, compute the weights of
                       the particles, resample the set, and update the private
                       member field that holds the current best pose estimate
                       (self.pose_estimate). Note that the motion parameters provided
                       to this function are in robot's reference frame.

         Hint: Check motion_model.py for documentation and example how to use it
               for the particle pose update.
               x, y, yaw -- are the odometry update of the robot's pose (increments)


         Remark: to compute the weight of particles, use the weigh_particles_callback member
                 field:

                     weights = self.weigh_particles_callback(particles_list)

         Remark: to generate a comletely random particle use the provided
                 random_particle_generator object:

                     particle = self.random_particle_generator.generate_particle()

         Finally  the best pose estimate and assign it to the corresponding member field
         for visualization:
                     self.pose_estimate = selected_particle.pose
        ============================================================================
        '''

        
        #Set motion with odometry increments
        self.motion_model.setMotion(x,y,yaw)     
        
                
        #Apply motion to particles
        for i in range(self.particle_set_size):
                self.particles[i].pose = self.motion_model.sample(self.particles[i].pose)
                
        #Get particle weights
        weights = self.weigh_particles_callback(self.particles)
           
        #Normalize
        weight_total = 0.0    
        for i in range(self.particle_set_size):
            weight_total += weights[i]
        
        for i in range(self.particle_set_size):
            self.particles[i].weight = weights[i]/weight_total

        #Sort particles
        self.particles.sort(reverse=True,key=lambda particles: particles.weight)
           
        #Resampling
        particles_resampled = self.particles
        particles_resampled = []
                
        #Stochastic universal sampling
        F = 1.0
        N = self.particle_set_size-self.random_particles_size
        P = F/N    
        Start = random.random()*P
        Points = []
        for i in range(N):
            Points.append(Start+i*P)
        fitness_sum = 0
        counter = 0  
        for j in range(len(Points)):
            while  fitness_sum < Points[j] and counter < len(Points):
                fitness_sum += self.particles[counter].weight
                counter += 1
            particles_resampled.append(self.particles[counter])

        
        self.particles = particles_resampled
        
        #Generate new samples
        for i in range(self.random_particles_size):
            self.particles.append(self.random_particle_generator.generate_particle())        
            rospy.loginfo(self.particles[-1].weight)

        #Pose estimate
        self.pose_estimate.theta +=  yaw
        self.pose_estimate.x += x*math.cos(self.pose_estimate.theta) - y*math.sin(self.pose_estimate.theta)
        self.pose_estimate.y += x*math.sin(self.pose_estimate.theta) + y*math.cos(self.pose_estimate.theta)


    def get_particles(self):
        return self.particles

    def get_pose_estimate(self):
        return self.pose_estimate

    def set_external_pose_estimate(self, pose):
        self.random_particle_generator.set_bias(pose, 0.5, 500)