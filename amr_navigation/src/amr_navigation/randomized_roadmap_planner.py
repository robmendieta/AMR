#!/usr/bin/env python


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

    def plan(self, point1, point2):
        """
        Plan a path which connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Return a list of tuples where each tuple represents a point in the
        planned path, the first point is the start point, and the last point is
        the end point. If the planning algorithm failed the returned list
        should be empty.
        """
        return list()

    def remove_edge(self, point1, point2):
        """
        Remove the edge of the graph that connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Has an effect only if both points have a corresponding node in the
        graph and if those nodes are connected by an edge.
        """
        pass

#==============================================================================
