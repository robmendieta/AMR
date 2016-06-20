import math
import random

from amr_localization.particle import Particle

# This class generates particles with random poses.
# By default the poses are drawn from a uniform distribution across the
# rectangle given in the constructor. Optinally a bias towards a particular
# pose may be introduced for a limited time (see @ref setBias()). */
class RandomParticleGenerator:
    def __init__(self, min_x, max_x, min_y, max_y):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.biased_particles = 0
        self,normalx = (0,0)
        self,normaly = (0,0)
        self,normalt = (0,0)

    # Generate a random particle
    def generate_particle(self):
        p = Particle()
        if self.biased_particles > 0:
            p.pose.x = random.normal(self.normalx)
            p.pose.y = random.normal(self.normaly)
            p.pose.theta = random.normal(self.normalt)
        else:
            p.pose.x = random.uniform(self.min_x, self.max_x)
            p.pose.y = random.uniform(self.min_y, self.max_y)
            p.pose.theta = random.uniform(-math.pi, math.pi)
        self.biased_particles -= 1
        p.weight = 0.0
        return p

    # Introduce a bias towards a certain point.
    # Here by bias we mean that the poses will be drawn from a normal
    # distribution around the bias pose.
    #
    # @param pose : a pose around which random samples will be drawn.
    #
    # @param std : standard deviation (same for x, y, and theta).
    #
    # @param particle_count : number of particles for which the bias will have
    # effect. After this many particles have been produced, the generator
    # switches to the "uniform" mode.

    def set_bias(self, pose, std, particle_count):
        self.normalx = (pose.x,std)
        self.normaly = (pose.y,std)
        self.normalt = (pose.theta,std)
        self.biased_particles = particle_count
