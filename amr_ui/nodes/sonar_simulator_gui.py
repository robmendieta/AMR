#!/usr/bin/env python

PACKAGE = 'amr_ui'
NODE = 'sonar_simulator_gui'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

import sys
import math
from functools import partial

from PySide.QtGui import *
from PySide.QtCore import *

import tf
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose2D
from amr_msgs.msg import Cone
from amr_srvs.srv import GetNearestOccupiedPointInCone, \
                        GetNearestOccupiedPointOnBeam


class Sonar:

    THRESHOLD = 50

    def __init__(self, caching=True):
        self.frame_id = 'sonar_simulated_link'
        self.beam_srv = rospy.ServiceProxy('/occupancy_query_server/'
                                           'get_nearest_occupied_point_on_beam',
                                           GetNearestOccupiedPointOnBeam)
        self.cone_srv = rospy.ServiceProxy('/occupancy_query_server/'
                                           'get_nearest_occupied_point_in_cone',
                                           GetNearestOccupiedPointInCone)
        self.range_pub = rospy.Publisher('sonar_simulated', Range)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.caching = caching
        self.set_parameters(0, 0, 0, 0, 0, 'beam')

    def set_parameters(self, x, y, yaw, fov, max_range, mode):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.fov = fov
        self.max_range = max_range
        self.position = (x, y, 0)
        self.orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        if mode == 'beam':
            beam = [Pose2D(x, y, yaw)]
            self.nearest_point = partial(self.beam_srv, beam, self.THRESHOLD)
        elif mode == 'cone':
            cone = [Cone(Pose2D(x, y, yaw), fov, max_range)]
            self.nearest_point = partial(self.cone_srv, cone, self.THRESHOLD)
        self.cache = None

    def publish(self):
        ts = rospy.Time.now()
        self.tf_broadcaster.sendTransform(self.position, self.orientation,
                                          ts, self.frame_id, 'odom')
        msg = Range()
        msg.header.stamp = ts
        msg.header.frame_id = self.frame_id
        msg.field_of_view = self.fov
        msg.max_range = self.max_range
        if self.caching and self.cache:
            msg.range = self.cache
        else:
            try:
                distance = self.nearest_point().distances[0]
                if distance > self.max_range:
                    msg.range = self.max_range
                else:
                    msg.range = distance
            except rospy.ServiceException:
                return
            self.cache = msg.range
        self.range_pub.publish(msg)


class Form(QDialog):
    def __init__(self, sonar, parent=None, defaults=None):
        super(Form, self).__init__(parent)
        self.sonar = sonar
        # Sonar parameters
        grid = QGridLayout()
        grid.addWidget(QLabel('X:'), 0, 0)
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-10, 10)
        self.x_spin.setSingleStep(0.1)
        if defaults and 'x' in defaults:
            self.x_spin.setValue(defaults['x'])
        self.x_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.x_spin, 0, 1)
        grid.addWidget(QLabel('Y:'), 1, 0)
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-10, 10)
        self.y_spin.setSingleStep(0.1)
        if defaults and 'y' in defaults:
            self.y_spin.setValue(defaults['y'])
        self.y_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.y_spin, 1, 1)
        grid.addWidget(QLabel('Yaw:'), 2, 0)
        self.yaw_spin = QSpinBox()
        self.yaw_spin.setRange(0, 360)
        if defaults and 'yaw' in defaults:
            self.yaw_spin.setValue(defaults['yaw'])
        self.yaw_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.yaw_spin, 2, 1)
        grid.addWidget(QLabel('FOV:'), 3, 0)
        self.fov_spin = QSpinBox()
        self.fov_spin.setRange(0, 45)
        if defaults and 'fov' in defaults:
            self.fov_spin.setValue(defaults['fov'])
        self.fov_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.fov_spin, 3, 1)
        grid.addWidget(QLabel('Max range:'), 4, 0)
        self.max_spin = QDoubleSpinBox()
        self.max_spin.setRange(0, 10)
        self.max_spin.setSingleStep(0.1)
        if defaults and 'max' in defaults:
            self.max_spin.setValue(defaults['max'])
        self.max_spin.valueChanged.connect(self.update_cb)
        grid.addWidget(self.max_spin, 4, 1)
        grid.addWidget(QLabel('Simulation mode:'), 5, 0)
        self.mode_box = QComboBox()
        self.mode_box.addItems(['beam', 'cone'])
        self.mode_box.currentIndexChanged.connect(self.update_cb)
        grid.addWidget(self.mode_box, 5, 1)
        self.setLayout(grid)
        self.update_cb(0)

    def update_cb(self, value):
        self.sonar.set_parameters(self.x_spin.value(), self.y_spin.value(),
                                  math.radians(self.yaw_spin.value()),
                                  math.radians(self.fov_spin.value()),
                                  self.max_spin.value(),
                                  self.mode_box.currentText())

if __name__ == '__main__':
    rospy.init_node(NODE)
    sonar = Sonar()
    app = QApplication(sys.argv)
    gui = Form(sonar, defaults={'yaw': 45, 'fov': 15, 'max': 5.0})
    gui.show()
    rospy.on_shutdown(lambda: app.exit())
    # Qt + Python + ROS hack: this timer will allow the interpreter to run each
    # 100 ms, publish the latest simulated sonar data, and at some point in
    # time receive the shutdown callback from ROS.
    timer = QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: sonar.publish())
    # Start application execution
    sys.exit(app.exec_())
