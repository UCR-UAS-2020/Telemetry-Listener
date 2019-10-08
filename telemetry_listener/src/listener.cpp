#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

void callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ROS_INFO("Latitude: [%f], Longitude: [%f], Altitude: [%f]", msg->latitude, msg->longitude, msg->altitude);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/mavros/global_position/global", 1000, callback);

  ros::spin();

  return 0;
}