#!/usr/bin/env python

PACKAGE = 'amr_ui'
NODE = 'occupancy_grid_display'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from nav_msgs.msg import OccupancyGrid
from subprocess import Popen, PIPE
import sys
import argparse


class Plot:
    def __init__(self, title, w, h):
        self.gnuplot = Popen(['gnuplot'], shell=False, stdin=PIPE, stderr=PIPE)
        self.gnuplot.stdin.write("set terminal wxt noraise size %i,%i" % (w, h))
        self.title = title

    def update(self, data, width):
        self.gnuplot.stdin.write("set title '%s'\n" % self.title)
        self.gnuplot.stdin.write("set pm3d map\n")
        self.gnuplot.stdin.write("splot '-' matrix\n")
        i = 0
        for d in data:
            self.gnuplot.stdin.write(" %i" % d)
            i += 1
            if i == width:
                i = 0
                self.gnuplot.stdin.write("\n")
        self.gnuplot.stdin.write("e\ne\n")

    def close(self):
        self.gnuplot.stdin.write("q")


if __name__ == '__main__':
    rospy.init_node(NODE, anonymous=True)
    parser = argparse.ArgumentParser(description='''
    Visualize occupancy grids with Gnuplot.

    This node subscribes to the map topic ('/map' by default) and displays the
    occupancy grids published there with Gnuplot pm3d (palette-mapped 3D)
    style. The target topic could be either remapped in a launch script or set
    via the '--topic' command-line argument. The other arguments control the
    title and the window size of the plot.
    ''', formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--title', help='plot title', default='Occupancy grid')
    parser.add_argument('--size', help='window size in pixels (format: xx,yy)')
    parser.add_argument('--topic', help='map topic name', default='/map')
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])
    try:
        w, h = (int(i) for i in args.size.split(','))
    except (ValueError, AttributeError):
        w, h = 300, 320
    rospy.loginfo('Creating Gnuplot window with title "%s" and size %ix%i.' %
                  (args.title, w, h))
    plot = Plot(args.title, w, h)

    def map_cb(msg):
        try:
            plot.update(msg.data, msg.info.width)
        except IOError:
            rospy.logerr('Gnuplot process has died, shutting down the node...')
            rospy.signal_shutdown('Gnuplot process has died')

    map_sub = rospy.Subscriber(args.topic, OccupancyGrid, map_cb)
    rospy.spin()
