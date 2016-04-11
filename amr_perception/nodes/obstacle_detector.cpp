#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

#include <amr_msgs/Ranges.h>
#include <amr_msgs/Obstacle.h>
#include <amr_srvs/SwitchRanger.h>
#include <amr_perception/ObstacleDetectorConfig.h>

ros::Subscriber sonar_subscriber;
ros::Subscriber velocity_subscriber;
ros::Publisher obstacle_publisher;

double safe_distance = 0.45;
std::vector<int> sonars;

void reconfigureCallback(amr_perception::ObstacleDetectorConfig &config, uint32_t level)
{
  safe_distance = config.safe_distance;
  ROS_INFO("Changed safe distance to: %.3f", config.safe_distance);
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double direction = atan2(msg->linear.y, msg->linear.x);
  if (msg->linear.y == 0 && msg->linear.x == 0)
    sonars = {};
  else if (direction < -2.97)
    sonars = {11, 12};
  else if (direction < -2.62)
    sonars = {7, 8, 9, 10, 11, 12, 13};
  else if (direction < -2.27)
    sonars = {6, 7, 8, 9, 10, 11, 12, 13};
  else if (direction < -1.57)
    sonars = {6, 7, 8, 9, 10, 11, 12};
  else if (direction < -0.87)
    sonars = {3, 4, 5, 6, 7, 8, 9};
  else if (direction < -0.52)
    sonars = {2, 3, 4, 5, 6, 7, 8, 9};
  else if (direction < -0.17)
    sonars = {2, 3, 4, 5, 6, 7, 8};
  else if (direction < 0.17)
    sonars = {3, 4};
  else if (direction < 0.52)
    sonars = {0, 1, 2, 3, 4, 5, 14, 15};
  else if (direction < 0.87)
    sonars = {14, 15, 0, 1, 2, 3, 4};
  else if (direction < 1.57)
    sonars = {11, 12, 13, 14, 15, 0, 1};
  else if (direction < 2.27)
    sonars = {10, 11, 12, 13, 14, 15, 0, 1};
  else if (direction < 2.62)
    sonars = {10, 11, 12, 13, 14, 15, 0};
  else
    sonars = {11, 12};
}

void sonarCallback(const amr_msgs::Ranges::ConstPtr& msg)
{
  for (const auto& s : sonars)
    if (msg->ranges[s].range < safe_distance)
    {
      amr_msgs::Obstacle obstacle;
      obstacle.header = msg->ranges[s].header;
      obstacle.distance = msg->ranges[s].range;
      obstacle_publisher.publish(obstacle);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detector");
  ros::NodeHandle nh;
  // Wait until SwitchRanger service (and hence stage node) becomes available.
  ROS_INFO("Waiting for the /switch_ranger service to be advertised...");
  ros::ServiceClient switch_ranger_client = nh.serviceClient<amr_srvs::SwitchRanger>("/switch_ranger");
  switch_ranger_client.waitForExistence();
  // Make sure that the pioneer sonars are available and enable them.
  amr_srvs::SwitchRanger srv;
  srv.request.name = "sonar_pioneer";
  srv.request.state = true;
  if (switch_ranger_client.call(srv))
  {
    ROS_INFO("Enabled pioneer sonars.");
  }
  else
  {
    ROS_ERROR("Pioneer sonars are not available, shutting down.");
    return 1;
  }
  // Create subscribers and publisher.
  sonar_subscriber = nh.subscribe("/sonar_pioneer", 100, sonarCallback);
  velocity_subscriber = nh.subscribe("/cmd_vel", 100, velocityCallback);
  obstacle_publisher = nh.advertise<amr_msgs::Obstacle>("/obstacles", 100);
  // Create dynamic reconfigure server.
  dynamic_reconfigure::Server<amr_perception::ObstacleDetectorConfig> server;
  server.setCallback(boost::bind(&reconfigureCallback, _1, _2));
  // Start infinite loop.
  ROS_INFO("Started obstacle detector node.");
  ros::spin();
  return 0;
}

