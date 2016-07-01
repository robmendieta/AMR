#!/usr/bin/env python

from pygraph.classes.graph import graph
from pygraph.classes.exceptions import NodeUnreachable
from pygraph.algorithms.heuristics.euclidean import euclidean
from pygraph.algorithms.minmax import heuristic_search
import random
import math
import rospy

#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty RandomizedRoadmapPlanner class.
#               An instance of this class will be created by the path_planner
#               node. It should maintain a graph of points and the connections
#               between them.
#               The 'plan()' function should find a path between the requested
#               points, inserting new nodes and edges if necessary. Make sure
#               that it stops at some point in time if no path between the
#               points exists.
#
# Remark: it will be necessary to test points and line segments for emptiness.
#         The class is (as usual) ROS-independent, so the actual mechanism of
#         performing these tests is abstracted by two callback functions, which
#         the object receives during the construction. In order to test whether
#         e.g. the point (1, 4) is free you should do:
#
#             free = self.point_free_cb((1, 4))
#
# Hint: use the standard function 'math.uniform()' to generate the coordinates
#       for random points.
#
# Hint: if you decided to use 'pygraph' library for graph and search
#       implementations, make sure that the graph object is stored in a member
#       field called 'graph'. If this is the case, the nodes and edges of the
#       graph will be automatically visualized by the path_planner node after
#       each planning request.

class RandomizedRoadmapPlanner:

    def __init__(self, point_free_cb, line_free_cb, dimensions):
        """
        Construct a randomized roadmap planner.

        'point_free_cb' is a function that accepts a point (two-tuple) and
        outputs a boolean value indicating whether the point is in free space.

        'line_free_cb' is a function that accepts two points (the start and the
        end of a line segment) and outputs a boolen value indicating whether
        the line segment is free from obstacles.

        'dimensions' is a tuple of tuples that define the x and y dimensions of
        the world, e.g. ((-8, 8), (-8, 8)). It should be used when generating
        random points.
        """

        self.point_free_cb = point_free_cb
        self.line_free_cb = line_free_cb
        self.dimensions = dimensions
        self.dimensions_x = self.dimensions[0]
        self.dimensions_y = self.dimensions[1]
        
        self.graph = graph()
        self.number_nodes = 5
        self.node_limit = 50
        self.node_counter = 0

    def plan(self, point1, point2):
        """
        Plan a path which connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Return a list of tuples where each tuple represents a point in the
        planned path, the first point is the start point, and the last point is
        the end point. If the planning algorithm failed the returned list
        should be empty.
        """
        path = []
        #Add start and end
        self.node_counter += 1
        start = self.node_counter
        self.graph.add_node(self.node_counter, attrs=[('position', (point1[0], point1[1]))])
        self.node_counter += 1
        end = self.node_counter
        self.graph.add_node(self.node_counter, attrs=[('position', (point2[0], point2[1]))])        

        new_node_counter = 0
        #Create nodes until a solution is found, limit = self.node_limit 
        while new_node_counter < self.node_limit :
            #Add 5 new random nodes each iteration
            for i in range(self.number_nodes):
                position_x = random.uniform(self.dimensions_x[0],self.dimensions_x[1])
                position_y = random.uniform(self.dimensions_y[0],self.dimensions_y[1])
                if self.point_free_cb((position_x,position_y)) :
                    self.node_counter += 1
                    new_node_counter += 1
                    self.graph.add_node(self.node_counter, attrs=[('position', (position_x, position_y))])
        
            #Add edges
            for nid1, attr in self.graph.node_attr.iteritems():
                position1 = attr[0][1]
                
                for nid2, attr2 in self.graph.node_attr.iteritems():
                    position2 = attr2[0][1]
                    distance = self.distance(position1, position2)
                    #Avoid self edges and repetitivity
                    if distance > 0.0 and self.graph.has_edge((nid1, nid2)) == False:                   
                        if self.line_free_cb(position1, position2):       
                            self.graph.add_edge((nid1, nid2), wt = distance)
            
            #A* search
            try:
                heuristic = euclidean()
                heuristic.optimize(self.graph)
                id_path = heuristic_search(self.graph, start, end, heuristic)
        
                for member in id_path:
                    attributes = self.graph.node_attributes(member)
                    path.append(attributes[0][1])            
            except NodeUnreachable:
                rospy.logwarn("Path unreachable, not enough nodes")
            else:
                return path
        
        #If node limit is passed, then no path is found
        rospy.logwarn("Node limit reached, path is not connected")
        return path
        
    def distance(self, point1, point2):
        distance = math.sqrt(math.pow(point1[0]-point2[0],2)+math.pow(point1[1]-point2[1],2))
        return distance

    def remove_edge(self, point1, point2):
        """
        Remove the edge of the graph that connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Has an effect only if both points have a corresponding node in the
        graph and if those nodes are connected by an edge.
        """
        pass

#==============================================================================
