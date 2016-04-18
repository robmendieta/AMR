#!/usr/bin/env python

PACKAGE = 'amr_navigation'

import math
from velocity_controller import VelocityController, Velocity
from velocity_controller import get_shortest_angle, get_distance

class OmniVelocityController(VelocityController):

    def __init__(self, *args):
        raise NotImplementedError('This is your assignment to implement OmniVelocityController')
        pass


    """
    ========================= YOUR CODE HERE =========================

    Instructions: put here all the functions that are necessary to
    implement the VelocityController interface. You may
    use the DiffVelocityController as an example.

    Implement the constructor to accept all the necessry parameters
    and implement compute_velocity() method

    You are free to write any helper functions or classes you might
    need.

    ==================================================================

    """