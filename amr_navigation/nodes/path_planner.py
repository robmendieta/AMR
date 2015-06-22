#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_planner'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
from amr_srvs.srv import PlanPath, IsPointFree, IsLineSegmentFree
from amr_msgs.msg import PathExecutionFailure
from amr_navigation.randomized_roadmap_planner import RandomizedRoadmapPlanner
from amr_navigation.graph_visualizer import GraphVisualizer


class PathPlannerNode:

    THRESHOLD = 50
    RADIUS = 0.26
    WIDTH = 0.50

    def __init__(self):
        # This will block until the stage node is loaded and has set the world
        # dimensions parameters
        while not (rospy.has_param('world_width') and
                   rospy.has_param('world_height')):
            continue
        ww = rospy.get_param('world_width')
        wh = rospy.get_param('world_height')
        dim = ((-ww / 2.0, ww / 2.0), (-wh / 2.0, wh / 2.0))

        self.srv = rospy.Service('~plan_path', PlanPath, self.plan_path_cb)
        self.failures_sub = rospy.Subscriber('~path_execution_failures',
                                             PathExecutionFailure,
                                             self.failure_cb)
        self.planner = RandomizedRoadmapPlanner(self.point_free_cb,
                                                self.line_free_cb, dim)
        self.visualizer = GraphVisualizer('path_planner', 'odom')
        self.point_srv = rospy.ServiceProxy('/occupancy_query_server/'
                                            'is_point_free',
                                            IsPointFree)
        self.line_srv = rospy.ServiceProxy('/occupancy_query_server/'
                                           'is_line_segment_free',
                                           IsLineSegmentFree)
        rospy.loginfo('Started [%s] node.' % (rospy.get_name()))

    def plan_path_cb(self, req):
        rospy.loginfo('Received [plan_path] request.')
        p1, p2 = (req.start.x, req.start.y), (req.end.x, req.end.y)
        path = self.planner.plan(p1, p2)
        try:
            self.visualizer.visualize(self.planner.graph)
        except:
            # If a student code in RandomizedRoadmapPlanner does not have a
            # graph object, or it is not compatible with GraphVisualizer,
            # silently skip visualization
            pass
        if not len(path):
            rospy.logwarn('Failed to find a path.')
            return None
        # Convert the path output by the planner to ROS message
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = rospy.Time.now()
        q_start = quaternion_from_euler(0, 0, req.start.theta)
        q_end = quaternion_from_euler(0, 0, req.end.theta)
        for pt in path:
            p = PoseStamped()
            p.pose.position = Point(pt[0], pt[1], 0.0)
            p.pose.orientation = Quaternion(*q_start)
            path_msg.poses.append(p)
        # The randomized roadmap planner does not consider orientation, thus
        # we manually add a final path segment which rotates the robot to the
        # desired orientation
        last = path_msg.poses[-1]
        last.pose.orientation = Quaternion(*q_end)
        path_msg.poses.append(last)
        return path_msg

    def failure_cb(self, msg):
        ps = msg.start.pose.position
        pe = msg.end.pose.position
        self.planner.remove_edge((ps.x, ps.y), (pe.x, pe.y))

    def point_free_cb(self, point):
        """
        This is a proxy function which uses the occupancy_query_server to
        test whether a particular point is free.
        """
        try:
            resp = self.point_srv(point[0], point[1], self.RADIUS,
                                  self.THRESHOLD)
            return resp.free
        except rospy.ServiceException:
            return False

    def line_free_cb(self, point1, point2):
        """
        This is a proxy function which uses the occupancy_query_server to
        test whether a particular line segment is free.
        """
        try:
            resp = self.line_srv(point1[0], point1[1], point2[0], point2[1],
                                 self.WIDTH, self.THRESHOLD)
            return resp.free
        except rospy.ServiceException:
            return False

if __name__ == '__main__':
    rospy.init_node(NODE)
    ppn = PathPlannerNode()
    rospy.spin()
