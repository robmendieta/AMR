#!/usr/bin/env python

PACKAGE = 'amr_ui'
NODE = 'pose_likelihood_display'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

import argparse
import sys
from numpy import arange, hstack
from itertools import product
from math import radians

from tf import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from std_msgs.msg import ColorRGBA
from amr_srvs.srv import GetMultiplePoseLikelihood, GetMultiplePoseLikelihoodRequest

if __name__ == '__main__':
    rospy.init_node(NODE)
    tf_listener = TransformListener(True, rospy.Duration(5))

    parser = argparse.ArgumentParser(description='''
    Visualize likelihoods of poses in a given area.

    This node will iterate over all possible poses (with a fixed orientation)
    in a given area of the world (with some discretisation, of course) and
    issue a [get_pose_likelihood] request for each of them. The results are
    visualized with an array of cube markers in RViz. Each cube is colored
    according to the likelihood of the corresponding pose, with red meaning
    very likely and black meaning very unlikely.

    The area where the likelihoods are evaluated is controlled by the '--area'
    option. If the ground truth pose of the robot is (x, y, theta), then the
    region will be [x - area; x + area] in x dimension and [y - area; y + area]
    in y dimension. The fixed orientation will be the true orientation plus the
    value of '--yaw' option.

    If the '--normalize' flag is set, then the computed likelihoods are scaled
    so that the pose with the maximum likelihood is painted red, and the pose
    with the minimum likelihood is painted black.
    ''', formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--yaw', help='yaw difference from the ground truth '
                        'orientation of the robot in degrees (default: 0)',
                        default=0.0, type=float)
    parser.add_argument('--area', help='defines the area of the world to test '
                        '(see description, default: 0.3)', default=0.3,
                        type=float)
    parser.add_argument('--step', help='discretization step in meters '
                        '(default: 0.05)', default=0.05, type=float)
    parser.add_argument('--normalize', action='store_true', help='scale '
                        'computed likelihoods (see description)')
    args = parser.parse_args()
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    marker_pub = rospy.Publisher('pose_likelihood', Marker)
    get_pose_likelihood = rospy.ServiceProxy('pose_likelihood_server/'
                                             'get_pose_likelihood',
                                             GetMultiplePoseLikelihood)

    rospy.sleep(1)
    time = tf_listener.getLatestCommonTime('odom', 'base_link')
    p, q = tf_listener.lookupTransform('odom', 'base_link', time)
    q = quaternion_from_euler(0, 0, euler_from_quaternion(q)[2] +
                              radians(args.yaw))

    def around(base, area, step):
        l = arange(base - step, base - area, -step)
        r = arange(base, base + area, step)
        return hstack([l, r])

    x_range = around(p[0], args.area, args.step)
    y_range = around(p[1], args.area, args.step)

    m = Marker()
    m.header.frame_id = 'odom'
    m.type = Marker.CUBE_LIST
    m.action = Marker.ADD
    m.scale.x = args.step
    m.scale.y = args.step
    m.scale.z = 0.05
    m.color.a = 0.8
    m.ns = 'likelihood'
    m.id = 0

    rospy.loginfo('Quering pose likelihoods, this may take a while...')
    scores = list()
    request = GetMultiplePoseLikelihoodRequest()
    for x, y in product(x_range, y_range):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = Quaternion(*q)
        request.poses.append(pose)
        m.points.append(Point(x=x, y=y))

    response = get_pose_likelihood(request)
    #rospy.loginfo(response)
    #rospy.loginfo(len(m.points))
    for i in range(len(m.points)):
        #rospy.loginfo("appending point: " + str(x) + " " + str(y))
        scores.append(response.likelihoods[i])
        i += 1

    #rospy.loginfo(scores)

    mx = max(scores) or 0.00001
    for s in scores:
        if args.normalize:
            m.colors.append(ColorRGBA(a=1.0, r=s/mx))
        else:
            m.colors.append(ColorRGBA(a=1.0, r=s))

    rospy.loginfo('Likelihoods: min %.5f | max %.5f' % (min(scores),
                                                        max(scores)))
    rospy.loginfo('Done, sending visualization message and exiting...')
    marker_pub.publish(m)
