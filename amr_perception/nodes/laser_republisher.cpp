#include <vector>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "amr_msgs/Ranges.h"

#include <amr_perception/LaserRepublisherConfig.h>


std::vector<double> sonar_ranges_front, sonar_ranges_rear;
/* the indices represent the virtual laser "cones" 
   Formula for cone angles: (index/150)*180, in degrees
*/
std::vector<int> laser_indices({149,118,101,84,67,50,33,1});
ros::Subscriber laser_front;
ros::Subscriber laser_rear;
ros::Publisher sonar_publisher;

void reconfigureCallback(amr_perception::LaserRepublisherConfig &config, uint32_t level)
{
  //rate = config.rate;
  //ROS_INFO("Changed sonar publishing rate to: %d", config.rate);
}

void scan_front(const sensor_msgs::LaserScan laser)
{
    sonar_ranges_front.clear();
    for (unsigned int i=0; i<laser_indices.size();i++)
    {
        sonar_ranges_front.push_back(laser.ranges [ laser_indices[i] ] );
    }   
}


void scan_rear(const sensor_msgs::LaserScan laser)
{
    sonar_ranges_rear.clear();
    for (unsigned int i=0; i<laser_indices.size();i++)
    {
        sonar_ranges_rear.push_back(laser.ranges [ laser_indices[i] ] );
    }   
}

void publish_sonar()
{
    if (!sonar_ranges_front.empty() && !sonar_ranges_rear.empty()) 
    {
        amr_msgs::Ranges msg_ranges;
        
        for (unsigned int i = 0; i < sonar_ranges_front.size(); i++)
        {
            sensor_msgs::Range msg_range;
            msg_range.range = sonar_ranges_front.at(i);
            msg_ranges.ranges.push_back(msg_range);
        }
        for (unsigned int i = 0; i < sonar_ranges_rear.size(); i++)
        {
            sensor_msgs::Range msg_range;
            msg_range.range = sonar_ranges_rear.at(i);
            msg_ranges.ranges.push_back(msg_range);
        }

        sonar_publisher.publish(msg_ranges);
        
    } else 
    {
        ROS_INFO("not able to publish sonar because one not all laser scans have arrived");
    }
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "laser_republisher");
  ros::NodeHandle n;

  laser_front = n.subscribe("/scan_front", 1, scan_front);
  laser_rear = n.subscribe("/scan_rear", 1, scan_rear);
  sonar_publisher = n.advertise<amr_msgs::Ranges>("/sonar_pioneer", 100);

  dynamic_reconfigure::Server<amr_perception::LaserRepublisherConfig> server;
  server.setCallback(boost::bind(&reconfigureCallback, _1, _2));

  ros::Rate rate(10); 
  while (ros::ok())
  {
    publish_sonar();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
