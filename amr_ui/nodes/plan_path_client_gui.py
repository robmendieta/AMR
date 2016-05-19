#!/usr/bin/env python

PACKAGE = 'amr_ui'
NODE = 'plan_path_client_gui'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

import sys
from PySide.QtGui import *
from PySide.QtCore import *

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from amr_srvs.srv import PlanPath


class Visualizer:
    def __init__(self):
        self.marker_pub = rospy.Publisher('path_endpoints', MarkerArray,
                                          latch=True)
        self.path_pub = rospy.Publisher('planned_path', Path)

    def points(self, points):
        ma = MarkerArray()
        for point in points:
            m = Marker()
            m.header.frame_id = 'odom'
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = 0.38
            m.scale.y = 0.38
            m.scale.z = 0.05
            m.ns = 'path_endpoints'
            m.id = len(ma.markers)
            m.pose.position.x = point[0]
            m.pose.position.y = point[1]
            m.pose.position.z = 0.05  # elevate above graph nodes' markers
            m.color = ColorRGBA(r=0.0, g=0.6, b=0.2, a=1.0)
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def path(self, path):
        self.path_pub.publish(path)


class Form(QDialog):
    def __init__(self, client, visualizer, parent=None, defaults=None):
        super(Form, self).__init__(parent)
        self.client = client
        self.visualizer = visualizer
        # General layout
        self.one_group = QGroupBox('Initial pose')
        self.two_group = QGroupBox('Target pose')
        self.buttons_grid = QGridLayout()
        layout = QVBoxLayout()
        layout.addWidget(self.one_group)
        layout.addWidget(self.two_group)
        layout.addLayout(self.buttons_grid)
        self.setLayout(layout)
        # Top -- initial pose group
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
        grid.addWidget(QLabel('Yaw:'), 2, 0)
        self.yaw1_spin = QSpinBox()
        self.yaw1_spin.setRange(0, 360)
        if defaults and 'yaw1' in defaults:
            self.yaw1_spin.setValue(defaults['yaw1'])
        grid.addWidget(self.yaw1_spin, 2, 1)
        self.one_group.setLayout(grid)
        # Middle -- target pose group
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
        grid.addWidget(QLabel('Yaw:'), 2, 0)
        self.yaw2_spin = QSpinBox()
        self.yaw2_spin.setRange(0, 360)
        if defaults and 'yaw2' in defaults:
            self.yaw2_spin.setValue(defaults['yaw2'])
        grid.addWidget(self.yaw2_spin, 2, 1)
        self.two_group.setLayout(grid)
        # Bottom -- plan button
        plan_button = QPushButton('Plan')
        plan_button.clicked.connect(self.button_plan_cb)
        self.buttons_grid.addWidget(plan_button, 0, 0)
        self.update_cb(0)

    def update_cb(self, value):
        self.visualizer.points(((self.x1_spin.value(), self.y1_spin.value()),
                                (self.x2_spin.value(), self.y2_spin.value())))

    def button_plan_cb(self):
        s = Pose2D(x=self.x1_spin.value(),
                   y=self.y1_spin.value(),
                   theta=self.yaw1_spin.value())
        e = Pose2D(x=self.x2_spin.value(),
                   y=self.y2_spin.value(),
                   theta=self.yaw2_spin.value())
        try:
            res = self.client(start=s, end=e)
        except rospy.ServiceException:
            rospy.logerr('Service call failed.')
            return
        self.visualizer.path(res.path)


if __name__ == '__main__':
    rospy.init_node(NODE)
    plan_client = rospy.ServiceProxy('path_planner/plan_path', PlanPath)
    app = QApplication(sys.argv)
    gui = Form(plan_client, Visualizer(), defaults={'x1': 7, 'y1': 2,
                                                    'x2': 1, 'y2': -5.5})
    gui.show()
    rospy.on_shutdown(lambda: app.exit())
    # Qt + Python hack: this timer will allow the interpreter to run each 500
    # ms and at some point in time receive the shutdown callback from ROS.
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)
    # Start application execution
    sys.exit(app.exec_())
