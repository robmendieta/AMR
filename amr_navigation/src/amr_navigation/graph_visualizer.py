#!/usr/bin/env python

PACKAGE = 'amr_navigation'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


class GraphVisualizer:
    """
    Given an instance of graph class from pygraph package, this class will
    visualize the nodes and the edges between them.
    The nodes are visualized with Sphere markers and the edges are visualized
    with Line markers in RViz. Each node should have a 'position' attribute.
    """
    def __init__(self, namespace, frame_id):
        self.edges_pub = rospy.Publisher('%s/edges' % namespace, Marker,
                                         latch=True)
        self.nodes_pub = rospy.Publisher('%s/nodes' % namespace, MarkerArray,
                                         latch=True)
        self.frame_id = frame_id

    def visualize(self, graph):
        # Create a dict where node ids are keys and positions are values
        nodes = dict()
        for nid, attr in graph.node_attr.iteritems():
            for k, v in attr:
                if k == 'position':
                    try:
                        nodes[nid] = point_to_msg(v)
                    except TypeError:
                        continue
        # Create and publish a marker array of spheres (which visualize nodes)
        ma = MarkerArray()
        for nid, position in nodes.iteritems():
            m = Marker()
            m.header.frame_id = self.frame_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = 0.38
            m.scale.y = 0.38
            m.scale.z = 0.05
            m.ns = 'nodes'
            m.id = len(ma.markers)
            m.pose.position = position
            m.color = ColorRGBA(r=0.8, g=0.0, b=0.1, a=1.0)
            ma.markers.append(m)
        self.nodes_pub.publish(ma)
        # Create and publish a marker with lines (which visualize edges)
        m = Marker()
        m.type = Marker.LINE_LIST
        m.header.frame_id = self.frame_id
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.pose.position.z = -0.1
        m.color = ColorRGBA(r=0.0, g=0.2, b=1.0, a=1.0)
        m.ns = 'lines'
        m.id = 0
        for n1, n2 in graph.edges():
            m.points.append(nodes[n1])
            m.points.append(nodes[n2])
        self.edges_pub.publish(m)


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
