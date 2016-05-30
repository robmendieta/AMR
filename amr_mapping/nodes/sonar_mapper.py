#!/usr/bin/env python

PACKAGE = 'amr_mapping'
NODE = 'sonar_mapper'

import rospy
import tf

from amr_srvs.srv import SwitchRanger, SwitchRangerRequest
from amr_msgs.msg import Ranges
from nav_msgs.msg import MapMetaData, OccupancyGrid
from amr_mapping.sonar_map import SonarMap


class SonarMapperNode:
    """
    This is a port of the AMR C++ SonarMapperNode
    """
    
    def __init__(self):
        
        rospy.init_node(NODE)
        
        # Wait until SwitchRanger service (and hence stage node) becomes available:
        rospy.loginfo('Waiting for the /switch_ranger service to be advertised...')
        switch_ranger_client = rospy.ServiceProxy('/switch_ranger', SwitchRanger)
        switch_ranger_client.wait_for_service()
        
        # Make sure that the braitenberg sonars are available and enable them.
        srr = SwitchRangerRequest()
        srr.name = 'sonar_pioneer'
        srr.state = True
        
        if switch_ranger_client.call(srr):
            rospy.loginfo('Enabled pioneer sonars.')
        else:
            rospy.logerr('Pioneer sonars are not available, shutting down.')
            exit()
        
        
        """
            Parameters
        """
        self._frame_id = rospy.get_param('frame_id', 'odom')
        resolution = rospy.get_param('sonar_mapper/resolution', 0.06)
        size_x = rospy.get_param('sonar_mapper/size_x', 16.0)
        size_y = rospy.get_param('sonar_mapper/size_y', 16.0)
        self._map_publication_period = rospy.get_param('sonar_mapper/map_publication_period', 5.0)
        
        self._map = SonarMap(resolution, size_x, size_y)
        
        
        """
            Publishers
        """
        self._map_publisher = rospy.Publisher('sonar_mapper/map',
                                              OccupancyGrid,
                                              queue_size=1)
        self._map_free_publisher = rospy.Publisher('sonar_mapper/map_free',
                                                   OccupancyGrid,
                                                   queue_size=1)
        self._map_occupied_publisher = rospy.Publisher('sonar_mapper/map_occupied',
                                                       OccupancyGrid,
                                                       queue_size=5)
        
        """
            Subscribers
        """
        self._sonar_subscriber = rospy.Subscriber('/sonar_pioneer',
                                                  Ranges,
                                                  self._sonar_callback,
                                                  queue_size=5)
        
        self._tf= tf.TransformListener()
        self._last_map_publication = rospy.Time.now()
        rospy.loginfo('Started [sonar_mapper] node.')
        self._publish_maps(True)
    
    
    def _sonar_callback(self, msg):
        count = 0
        for r in msg.ranges:
            count += 1
            try:
                time = self._tf.getLatestCommonTime(self._frame_id,
                                                    r.header.frame_id)
                position, quaternion = self._tf.lookupTransform(self._frame_id,
                                                                r.header.frame_id,
                                                                [r.header.stamp-rospy.Duration(0.01), time][time<r.header.stamp])
            except Exception as ex:
                rospy.logwarn('Unable to incorporate sonar reading in the map '
                              'Reason: {}.'.format(ex.message))
                continue
            #Incorporate range reading in the map:
            self._map.add_scan(position[0],
                               position[1],
                               tf.transformations.euler_from_quaternion(quaternion)[2],
                               r.field_of_view,
                               r.max_range,
                               r.range,
                               self._calculate_range_uncertainty(r.range, r.max_range))
        self._publish_maps()
    
    
    def _publish_maps(self, force = False):
        if force or self._last_map_publication + rospy.Duration(self._map_publication_period)<= rospy.Time.now():
            w, h = self._map.get_grid_size_x(), self._map.get_grid_size_y()
            min_x, min_y = self._map.get_min_x(), self._map.get_min_y()
            resolution = self._map.get_resolution()
            self._map_publisher.publish(self._create_occupancy_grid_message(
                                        w, h, resolution, min_x, min_y, -1.0, 1.0,
                                        self._map.get_map_data()))
            self._map_free_publisher.publish(self._create_occupancy_grid_message(
                                        w, h, resolution, min_x, min_y, 0.0, 1.0,
                                        self._map.get_map_free_data()))
            self._map_occupied_publisher.publish(self._create_occupancy_grid_message(
                                        w, h, resolution, min_x, min_y, 0.0, 1.0,
                                        self._map.get_map_occupied_data()))
            self._last_map_publication = rospy.Time.now()
    
    
    def _create_occupancy_grid_message(self, width, height,
                                       resolution,
                                       orig_x, orig_y,
                                       min_val, max_val,
                                       data):
        EPSILON = 1e-5
        grid_msg = OccupancyGrid()
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.resolution = resolution
        grid_msg.info.map_load_time = rospy.Time.now()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = self._frame_id
        grid_msg.info.origin.position.x = orig_x
        grid_msg.info.origin.position.y = orig_y
        grid_msg.data = data
        
        return grid_msg
    
    
    def _calculate_range_uncertainty(self, registered_range, max_range):
        if registered_range<0.1*max_range:
            return 0.01*max_range
        elif registered_range<0.5*max_range:
            return 0.1*registered_range
        else:
            return 0.05*max_range


if __name__ == '__main__':
    n = SonarMapperNode()
    rospy.spin()
