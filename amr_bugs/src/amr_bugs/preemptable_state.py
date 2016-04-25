PACKAGE = 'amr_bugs'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import smach


class PreemptableState(smach.State):
    """
    Turn a function into a smach state with preemption.
    The provided callback function will be executed in an infinite loop at the
    given frequency until ROS is shut down, preemption is requested, or the
    function returns a valid outcome string.
    The 'preempted' outcome is returned if the preemption happens or if ROS is
    shut down. This outcome is appended to the list of possible outcomes
    automatically.
    """
    def __init__(self, callback, input_keys, output_keys, outcomes, freq=10):
        self.callback = callback
        self.frequency = freq
        smach.State.__init__(self,
                             input_keys=input_keys,
                             output_keys=output_keys,
                             outcomes=(outcomes + ['preempted']))

    def execute(self, userdata):
        r = rospy.Rate(self.frequency)
        while not rospy.is_shutdown() and not self.preempt_requested():
            result = self.callback(userdata)
            if result in self._outcomes:
                return result
            r.sleep()
        self.service_preempt()
        return 'preempted'
