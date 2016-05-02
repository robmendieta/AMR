#!/usr/bin/env python

PACKAGE = 'amr_bugs'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


class BugBrainVisualizer:
    """
    Given an instance of BugBrain class (or, in fact, any class), it will
    periodically check all the fields of that instance to find those that start
    with 'wp_' or 'ln_'. The former are considered to be waypoints, and the
    latter are lines. If the objects stored in these fields are proper points
    or lines, they get visualized.
    By proper point we mean a tuple/list of two numbers or an object that has
    'x' and 'y' attributes.
    By proper line we mean a tuple/list of two points, or an object of type
    planar.c.Line.
    """
    def __init__(self, brain):
        self.brain = brain
        self.lines_pub = rospy.Publisher('bug2/lines', Marker)
        self.waypoints_pub = rospy.Publisher('bug2/waypoints', MarkerArray)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        self.cm = ColorManager()

    def timer_cb(self, event):
        self.publish_lines()
        self.publish_waypoints()

    def publish_waypoints(self):
        ma = MarkerArray()
        for k, v in self.brain.__dict__.iteritems():
            if k.startswith('wp_'):
                m = Marker()
                m.header.frame_id = 'odom'
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.scale.x = 0.4
                m.scale.y = 0.4
                m.scale.z = 0.05
                m.ns = 'waypoints'
                m.id = len(ma.markers)
                try:
                    m.pose.position = point_to_msg(v)
                    m.color = self.cm.get_color(k)
                except TypeError:
                    continue
                ma.markers.append(m)
        self.waypoints_pub.publish(ma)

    def publish_lines(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.color.a = 1.0
        m.ns = 'lines'
        m.id = 0
        for k, v in self.brain.__dict__.iteritems():
            if k.startswith('ln_'):
                try:
                    p1, p2 = line_to_msgs(v)
                    m.points.append(p1)
                    m.points.append(p2)
                    m.colors.append(self.cm.get_color(k))
                    m.colors.append(self.cm.get_color(k))
                except TypeError:
                    pass
        self.lines_pub.publish(m)

    def shutdown(self):
        self.timer.shutdown()


class ColorManager:

    COLORS = [(0.8, 0.0, 0.2),
              (0.6, 0.6, 0.0),
              (0.2, 0.2, 0.8),
              (0.0, 0.6, 0.2),
              (0.9, 0.0, 0.6),
              (0.1, 1.0, 0.2),
              (0.1, 0.3, 0.7)]

    def __init__(self):
        self.objects = dict()

    def get_color(self, obj):
        if obj in self.objects:
            return self.objects[obj]
        else:
            c = ColorRGBA()
            c.r, c.g, c.b = self.COLORS[len(self.objects) % len(self.COLORS)]
            c.a = 1.0
            self.objects[obj] = c
            return c


def point_to_msg(v):
    p = Point()
    if hasattr(v, 'x') and hasattr(v, 'y'):
        p.x, p.y = v.x, v.y
    elif isinstance(v, tuple) and len(v) == 2:
        p.x, p.y = v
    else:
        raise TypeError('Argument is not a point-like object')
    p.z = 0.0
    return p


def line_to_msgs(v):
    if hasattr(v, 'direction'):
        d = v.direction.scaled_to(10)
        c = v.normal * v.offset
        return point_to_msg(c + d), point_to_msg(c - d)
    elif (isinstance(v, list) or isinstance(v, tuple)) and len(v) == 2:
        return point_to_msg(v[0]), point_to_msg(v[1])
    else:
        raise TypeError('Argument is not a line-like object')
