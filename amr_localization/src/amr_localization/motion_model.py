import random
import math
from amr_localization.pose import Pose

# This class represents a motion model for omnidirectional robot and could be
# used to sample the possible pose given the starting pose and the commanded
# robot's motion.
#
# The two parameters of the class is standard deviations of translational and
# rotational components of the motion.
#
# The motion is decomposed into two translations along the x axis of the
# robot (forward), and along the y axis of the robot (lateral), and one
# rotation.
#
# Usage:
#
# @code
#   // Create motion model with 0.02 and 0.01 stddev
#   motion_model = MotionModel(0.02, 0.01);
#   // Set the commanded robot's motion
#   motion_model.setMotion(0.5, 0.1, 0.1);
#   // Sample the possible pose given the starting pose
#   // Note that it could be repeated multiple times for the same starting
#   // pose of for different starting poses
#   new_pose = motion_model.sample(pose);
# @code

class MotionModel:
    def __init__(self, sigma_translation, sigma_rotation):
        self.forward = 0
        self.lateral = 0
        self.rotation = 0
        self.sigtran = sigma_translation
        self.sigrot = sigma_rotation

    def setMotion(self, forward, lateral, rotation):
        self.forward = forward
        self.lateral = lateral
        self.rotation = rotation

    def sample(self, pose):
        """
        Sample a possible pose resulting from the commanded robot's motion, if
        the robot was in given pose.
        """

        """
        ============================== YOUR CODE HERE ============================
         Instructions: given the starting pose, compute the new pose according to
                       the motion model. Note that both input and output pose are
                       in world coordinate frame, but the motion parameters are
                       in robot's reference frame.

         Hint: there are two member fields that represent translational and
               rotational error distributions. For example, to get a random
               translational error use:

               error = random.gauss(0, self.sigtran)

        ==========================================================================
        """

        return Pose()