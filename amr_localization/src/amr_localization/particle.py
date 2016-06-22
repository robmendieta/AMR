#!/usr/bin/env python

from amr_localization.pose import Pose

# Representation of a weighted particle for the particle filter.
# This structure represents a particle of the particle filter, composed of the
# robot's assumed pose and the weight assigned to this pose based on
# observation match.

class Particle:
    def __init__(self):
        # Robot pose proposed by this particle.
        self.pose = Pose()

        # Importance of this particle after observation update.
        # Measured as match between real observation and simulated observation from
        # this particle's pose. This should be a positive value between 0 and some
        # limit @c max_weight, such that a value close to zero indicate bad match
        # with real observation and a value close to @c max_weight indicates almost
        # perfect match.
        self.weight = 0

    # Particle comparison operator (based on weight).
    # Implementing this operator allows to use standard library algorithms to
    # find minimum and maximum elements in a vector of Particles, or sort it.
    def __lt__(self, rhs):
        return self.weight < rhs.weight