#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <nav_msgs/OccupancyGrid.h>

#include <amr_srvs/GetNearestOccupiedPointInCone.h>
#include <amr_srvs/GetNearestOccupiedPointOnBeam.h>
#include <amr_srvs/IsPointFree.h>
#include <amr_srvs/IsLineSegmentFree.h>

#include "map_store_cone.h"
#include "map_store_beam.h"
#include "map_store_circle.h"

class OccupancyQueryServerNode
{

public:

  OccupancyQueryServerNode()
  {
    ros::NodeHandle pn("~");
    map_subscriber_ = nh_.subscribe("/map", 1, &OccupancyQueryServerNode::mapCallback, this);
    cone_server_ = pn.advertiseService("get_nearest_occupied_point_in_cone", &OccupancyQueryServerNode::coneCallback, this);
    beam_server_ = pn.advertiseService("get_nearest_occupied_point_on_beam", &OccupancyQueryServerNode::beamCallback, this);
    point_server_ = pn.advertiseService("is_point_free", &OccupancyQueryServerNode::pointCallback, this);
    segment_server_ = pn.advertiseService("is_line_segment_free", &OccupancyQueryServerNode::segmentCallback, this);
    ROS_INFO("Started [occupancy_query_server] node.");
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    map_ = msg;
    ROS_INFO("Received a map.");
  }

  bool coneCallback(amr_srvs::GetNearestOccupiedPointInCone::Request& request, amr_srvs::GetNearestOccupiedPointInCone::Response& response)
  {
    ROS_INFO("Received [get_nearest_occupied_point_in_cone] request.");

    if (!map_)
    {
      ROS_WARN("Unable to answer the query because the map is not available.");
      return false;
    }

    // Prepare response arrays
    size_t num_cones = request.cones.size();
    response.distances.resize(num_cones);
    response.points.resize(num_cones);

    for (size_t i = 0; i < num_cones; i++)
    {
      // Set the default output
      response.distances[i] = std::numeric_limits<double>::infinity();
      response.points[i].x = std::numeric_limits<double>::quiet_NaN();
      response.points[i].y = std::numeric_limits<double>::quiet_NaN();

      const auto& cone = request.cones[i];
      if (cone.width <= 0 || cone.length <= 0)
      {
        ROS_WARN("Cone %zu has invalid width or length, skipping it.", i);
        continue;
      }

      int cone_x, cone_y, cell_x, cell_y;
      if (!convertToCell(cone.pose.x, cone.pose.y, cone_x, cone_y))
      {
        ROS_WARN("Origin of cone %zu is outside of the map, skipping it.", i);
        continue;
      }

      mapstore::MapStoreCone cone_cells(cone_x, cone_y, cone.pose.theta, cone.width, lround(cone.length / map_->info.resolution));
      while (cone_cells.nextCell(cell_x, cell_y))
      {
        double x, y;
        if (convertToMap(cell_x, cell_y, x, y))
        {
          int p = map_->data[cell_y * map_->info.width + cell_x];
          if (p >= request.threshold)
          {
            double distance = computeEuclideanDistance(cone.pose.x, cone.pose.y, x, y);
            if (distance < response.distances[i])
            {
              response.distances[i] = distance;
              response.points[i].x = x;
              response.points[i].y = y;
            }
          }
        }
      }
    }

    return true;
  }

  bool beamCallback(amr_srvs::GetNearestOccupiedPointOnBeam::Request& request, amr_srvs::GetNearestOccupiedPointOnBeam::Response& response)
  {
    ROS_INFO("Received [get_nearest_occupied_point_on_beam] request.");

    if (!map_)
    {
      ROS_WARN("Unable to answer the query because the map is not available.");
      return false;
    }

    // Prepare response arrays
    size_t num_beams = request.beams.size();
    response.distances.resize(num_beams);
    response.points.resize(num_beams);

    for (size_t i = 0; i < num_beams; i++)
    {
      // Set the default output
      response.distances[i] = std::numeric_limits<double>::infinity();
      response.points[i].x = std::numeric_limits<double>::quiet_NaN();
      response.points[i].y = std::numeric_limits<double>::quiet_NaN();

      const auto& beam = request.beams[i];

      double beam_x, beam_y;
      if (!convertToCell(beam.x, beam.y, beam_x, beam_y))
      {
        ROS_WARN("Origin of beam %zu is outside of the map, skipping it.", i);
        continue;
      }

      int cell_x, cell_y;
      mapstore::MapStoreBeam beam_cells(beam_x, beam_y, beam.theta, 0.0);
      while (beam_cells.nextCell(cell_x, cell_y))
      {
        double x, y;
        if (convertToMap(cell_x, cell_y, x, y))
        {
          int p = map_->data[cell_y * map_->info.width + cell_x];
          if (p >= request.threshold)
          {
            double distance = computeEuclideanDistance(beam.x, beam.y, x, y);
            if (distance < response.distances[i])
            {
              response.distances[i] = distance;
              response.points[i].x = x;
              response.points[i].y = y;
            }
          }
        }
        else
        {
          break;
        }
      }
    }

    return true;
  }

  bool pointCallback(amr_srvs::IsPointFree::Request& request, amr_srvs::IsPointFree::Response& response)
  {
    ROS_INFO("Received [is_point_free] request.");

    if (!map_)
    {
      ROS_WARN("Unable to answer the query because the map is not available.");
      return false;
    }

    double circle_x, circle_y;
    if (!convertToCell(request.x, request.y, circle_x, circle_y))
    {
      ROS_WARN("Requested point is outside of the map.");
      return false;
    }

    mapstore::MapStoreCircle circle(circle_x, circle_y, lround(request.diameter / 2.0 / map_->info.resolution));
    int cell_x, cell_y;
    while (circle.nextCell(cell_x, cell_y))
    {
      double x, y;
      if (convertToMap(cell_x, cell_y, x, y))
      {
        int p = map_->data[cell_y * map_->info.width + cell_x];
        if (p >= request.threshold)
        {
          response.free = false;
          return true;
        }
      }
    }
    response.free = true;
    return true;
  }

  bool segmentCallback(amr_srvs::IsLineSegmentFree::Request& request, amr_srvs::IsLineSegmentFree::Response& response)
  {
    ROS_INFO("Received [is_line_segment_free] request.");

    if (!map_)
    {
      ROS_WARN("Unable to answer the query because the map is not available.");
      return false;
    }

    double x1, y1, x2, y2;
    if (!convertToCell(request.x1, request.y1, x1, y1) ||
        !convertToCell(request.x2, request.y2, x2, y2))
    {
      ROS_WARN("One of the segment vertices is outside of the map.");
      return false;
    }

    int cell_x, cell_y;
    size_t num_beams = ceil(request.width / map_->info.resolution) + 1;
    double length = computeEuclideanDistance(request.x1, request.y1, request.x2, request.y2) / map_->info.resolution;
    double direction = atan2(request.y2 - request.y1, request.x2 - request.x1);
    double p_x = x1 - sin(direction) * request.width / 2 / map_->info.resolution;
    double p_y = y1 + cos(direction) * request.width / 2 / map_->info.resolution;
    double step_x = sin(direction) * request.width / (num_beams - 1) / map_->info.resolution;
    double step_y = -cos(direction) * request.width / (num_beams - 1) / map_->info.resolution;
    for (size_t i = 0; i < num_beams; i++)
    {
      mapstore::MapStoreBeam beam(p_x, p_y, direction, length);
      while (beam.nextCell(cell_x, cell_y))
      {
        double x, y;
        if (convertToMap(cell_x, cell_y, x, y))
        {
          int p = map_->data[cell_y * map_->info.width + cell_x];
          if (p >= request.threshold)
          {
            response.free = false;
            return true;
          }
        }
      }
      p_x += step_x;
      p_y += step_y;
    }
    response.free = true;
    return true;
  }

private:

  /** Determine the grid cell that contains the point given by a map coordinate.
    *
    * @return flag if the coordinate (@a m_x, @a m_y) is in the map. If the
    * coordinate is outside the map, then @a c_x and @a c_y are not valid cell
    * coordinates for this map. */
  bool convertToCell(const double m_x, const double m_y, int &c_x, int &c_y) const
  {
    ROS_ASSERT(map_);
    c_x = lround((m_x - map_->info.origin.position.x) / map_->info.resolution);
    c_y = lround((m_y - map_->info.origin.position.y) / map_->info.resolution);
    return (c_x >= 0 && std::abs(c_x) < map_->info.width &&
            c_y >= 0 && std::abs(c_y) < map_->info.height);
  }

  /** Determine the grid cell that contains the point given by a map coordinate.
    *
    * This version does not round the grid cell coordinates.
    *
    * @return flag if the coordinate (@a m_x, @a m_y) is in the map. If the
    * coordinate is outside the map, then @a c_x and @a c_y are not valid cell
    * coordinates for this map. */
  bool convertToCell(const double m_x, const double m_y, double &c_x, double &c_y) const
  {
    ROS_ASSERT(map_);
    c_x = (m_x - map_->info.origin.position.x) / map_->info.resolution;
    c_y = (m_y - map_->info.origin.position.y) / map_->info.resolution;
    return (c_x >= 0 && (size_t)lround(std::abs(c_x)) < map_->info.width &&
            c_y >= 0 && (size_t)lround(std::abs(c_y)) < map_->info.height);
  }

  /** Determine the map coordinates given map cell.
    *
    * The computed map coordinates correspond to the center of the cell.
    *
    * @return flag if the cell with index (@a c_x, @a c_y) is in the map. If the
    * cell is outside the map, then @a m_x and @a m_y are not valid map
    * coordinates for this map. */
  bool convertToMap(const int c_x, const int c_y, double &m_x, double &m_y) const
  {
    ROS_ASSERT(map_);
    m_x = (0.5 + c_x) * map_->info.resolution + map_->info.origin.position.x;
    m_y = (0.5 + c_y) * map_->info.resolution + map_->info.origin.position.y;
    return (c_x >= 0 && std::abs(c_x) < map_->info.width &&
            c_y >= 0 && std::abs(c_y) < map_->info.height);
  }

  /** Determine the map coordinates given map cell.
    *
    * @return flag if the cell with index (@a c_x, @a c_y) is in the map. If the
    * cell is outside the map, then @a m_x and @a m_y are not valid map
    * coordinates for this map. */
  bool convertToMap(const double c_x, const double c_y, double &m_x, double &m_y) const
  {
    ROS_ASSERT(map_);
    m_x = c_x * map_->info.resolution + map_->info.origin.position.x;
    m_y = c_y * map_->info.resolution + map_->info.origin.position.y;
    return (c_x >= 0 && std::abs(c_x) < map_->info.width &&
            c_y >= 0 && std::abs(c_y) < map_->info.height);
  }

  double computeEuclideanDistance(double x1, double y1, double x2, double y2) const
  {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  ros::NodeHandle nh_;
  ros::Subscriber map_subscriber_;
  ros::ServiceServer cone_server_;
  ros::ServiceServer beam_server_;
  ros::ServiceServer beam_batch_server_;
  ros::ServiceServer point_server_;
  ros::ServiceServer segment_server_;

  nav_msgs::OccupancyGridConstPtr map_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_query_server");
  OccupancyQueryServerNode oqsn;
  ros::spin();
  return 0;
}
