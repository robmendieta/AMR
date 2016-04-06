#!/usr/bin/env python

PACKAGE = 'amr_ui'
NODE = 'occupancy_query_client_gui'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

import sys
from PySide.QtGui import *
from PySide.QtCore import *

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from amr_srvs.srv import IsPointFree, IsLineSegmentFree


class Visualizer:

    THRESHOLD = 50

    def __init__(self):
        self.marker_pub = rospy.Publisher('occupancy_marker', Marker)
        self.color_free = ColorRGBA()
        cf = ColorRGBA()
        cf.r, cf.g, cf.b, cf.a = 0.0, 0.6, 0.2, 1.0
        self.color_free = cf
        co = ColorRGBA()
        co.r, co.g, co.b, co.a = 0.8, 0.0, 0.2, 1.0
        self.color_occupied = co
        self.point_srv = rospy.ServiceProxy('/occupancy_query_server/'
                                            'is_point_free',
                                            IsPointFree)
        self.line_srv = rospy.ServiceProxy('/occupancy_query_server/'
                                            'is_line_segment_free',
                                            IsLineSegmentFree)

    def point(self, x, y, radius):
        try:
            resp = self.point_srv(x, y, radius, self.THRESHOLD)
        except rospy.ServiceException:
            return
        m = Marker()
        m.header.frame_id = 'odom'
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = radius
        m.scale.y = radius
        m.scale.z = 0.05
        m.color = self.color_free if resp.free else self.color_occupied
        m.ns = 'occupancy'
        m.id = 0
        m.pose.position.x = x
        m.pose.position.y = y
        self.marker_pub.publish(m)

    def line(self, x1, y1, x2, y2, width):
        try:
            resp = self.line_srv(x1, y1, x2, y2, width, self.THRESHOLD)
        except rospy.ServiceException:
            return
        m = Marker()
        m.header.frame_id = 'odom'
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = width
        m.scale.y = 0.1
        m.color = self.color_free if resp.free else self.color_occupied
        m.ns = 'occupancy'
        m.id = 0
        p1 = Point()
        p1.x, p1.y = x1, y1
        p2 = Point()
        p2.x, p2.y = x2, y2
        m.points.append(p1)
        m.points.append(p2)
        self.marker_pub.publish(m)


class Form(QDialog):
    def __init__(self, visualizer, parent=None, defaults=None):
        super(Form, self).__init__(parent)
        self.visualizer = visualizer
        # General layout
        self.opt_grid = QGridLayout()
        self.one_group = QGroupBox('Point 1')
        self.two_group = QGroupBox('Point 2')
        layout = QVBoxLayout()
        layout.addLayout(self.opt_grid)
        layout.addWidget(self.one_group)
        layout.addWidget(self.two_group)
        self.setLayout(layout)
        # Top -- options grid
        self.opt_grid.addWidget(QLabel('Query mode:'), 0, 0)
        self.mode_box = QComboBox()
        self.mode_box.addItems(['point', 'line segment'])
        self.mode_box.currentIndexChanged.connect(self.mode_cb)
        self.opt_grid.addWidget(self.mode_box, 0, 1)
        self.opt_grid.addWidget(QLabel('Diameter/width:'), 1, 0)
        self.width_spin = QDoubleSpinBox()
        self.width_spin.setRange(0.0, 3)
        self.width_spin.setSingleStep(0.01)
        if defaults and 'width' in defaults:
            self.width_spin.setValue(defaults['width'])
        self.width_spin.valueChanged.connect(self.update_cb)
        self.opt_grid.addWidget(self.width_spin, 1, 1)
        # Middle -- point 1 group
        grid = QGridLayout()
        grid.addWidget(QLabel('X:'), 0, 0)
        self.x1_spin = QDoubleSpinBox()
        self.x1_spin.setRange(-10, 10)
        self.x1_spin.setSingleStep(0.1)
        if defaults and 'x1' in defaults:
            self.x1_spin.setValue(defaults['x1'])
        self.x1_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.x1_spin, 0, 1)
        grid.addWidget(QLabel('Y:'), 1, 0)
        self.y1_spin = QDoubleSpinBox()
        self.y1_spin.setRange(-10, 10)
        self.y1_spin.setSingleStep(0.1)
        if defaults and 'y1' in defaults:
            self.y1_spin.setValue(defaults['y1'])
        self.y1_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.y1_spin, 1, 1)
        self.one_group.setLayout(grid)
        # Bottom -- point 2 group
        grid = QGridLayout()
        grid.addWidget(QLabel('X:'), 0, 0)
        self.x2_spin = QDoubleSpinBox()
        self.x2_spin.setRange(-10, 10)
        self.x2_spin.setSingleStep(0.1)
        if defaults and 'x2' in defaults:
            self.x2_spin.setValue(defaults['x2'])
        self.x2_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.x2_spin, 0, 1)
        grid.addWidget(QLabel('Y:'), 1, 0)
        self.y2_spin = QDoubleSpinBox()
        self.y2_spin.setRange(-10, 10)
        self.y2_spin.setSingleStep(0.1)
        if defaults and 'y2' in defaults:
            self.y2_spin.setValue(defaults['y2'])
        self.y2_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.y2_spin, 1, 1)
        self.two_group.setLayout(grid)
        self.two_group.setDisabled(True)
        self.update_cb(0)

    def update_cb(self, value):
        if self.mode_box.currentText() == 'point':
            self.visualizer.point(self.x1_spin.value(), self.y1_spin.value(),
                                  self.width_spin.value())
        else:
            self.visualizer.line(self.x1_spin.value(), self.y1_spin.value(),
                                 self.x2_spin.value(), self.y2_spin.value(),
                                 self.width_spin.value())

    def mode_cb(self, value):
        if value == 'point':
            self.two_group.setDisabled(True)
        else:
            self.two_group.setDisabled(False)
        self.update_cb(0)


if __name__ == '__main__':
    rospy.init_node(NODE)
    app = QApplication(sys.argv)
    gui = Form(Visualizer(), defaults={'x2': 1, 'y2': 1, 'width': 0.25})
    gui.show()
    rospy.on_shutdown(lambda: app.exit())
    # Qt + Python hack: this timer will allow the interpreter to run each 500
    # ms and at some point in time receive the shutdown callback from ROS.
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)
    # Start application execution
    sys.exit(app.exec_())
