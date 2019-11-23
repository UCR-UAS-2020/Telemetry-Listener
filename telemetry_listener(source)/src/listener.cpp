// ROS headers
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
// C library headers
#include <cmath>

void callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ROS_INFO("Latitude: [%f], Longitude: [%f], Altitude: [%f]", msg->latitude, msg->longitude, msg->altitude);
  float Tlat_D = msg->latitude; //Target latitude in degrees
  float Tlon_D = msg->longitude; //Target longitude in degrees
  float Talt = msg->altitude; //Target altitude
  float GSlat_D = 33.975644;  //temporary values until we can connect with a sensor
  float GSlon_D = -117.325958;
  float GSalt = 0.0;
  float Tlat_R = Tlat_D*(M_PI/180); //Target latitude in radians
  float Tlon_R = Tlon_D*(M_PI/180); //Target longitude in radians
  float GSlat_R = GSlat_D*(M_PI/180); //Ground station latitude in radians
  float GSlon_R = GSlon_D*(M_PI/180); //Ground station longitude in radians
  float DLAT_D = Tlat_D - GSlat_D;  //Difference in latitude between target and ground station (Degrees)
  float DLON_D = Tlon_D - GSlon_D;  //Difference in laongitude between target and ground station (Degrees)
  float DLAT_R = Tlat_R - GSlat_R;  //Difference in latitude between target and ground station (Radians)
  float DLON_R = Tlon_R - GSlon_R;  //Difference in longitude between target and ground station (Radians)

  // Convert separation in lat/lon from degrees to meters
    //Average of latitudes (used as reference latitude for converting decimal degrees to distance in meters)
      float lat_ave = (Tlat_R + GSlat_R)/2.0;
    //Parameters for the shape of the Earth, in meters
      float a = 6378137; //Equatorial radius
      float e = 0.0818192; //eccentricity
    //Calculate distance in meters for 1 degree of latitude at this location (use chord length instead of arc length?)
      //Arc distance for 1 degree in meters
        float d1 = (M_PI * a * (1 - pow(e,2))) / (180 * pow((1-(pow(e,2)*pow(sin(lat_ave),2))),3/2));
    //Total distance in meters (y-direction)
      float dy = fabs(d1 * DLAT_D);
    //Calculate distance in meters for 1 degree of longitude at this location
      //Arc distance for 1 degree in meters
        float d2 = (M_PI * a * cos(lat_ave)) / (180 * sqrt(1 - (pow(e,2) * pow(sin(lat_ave),2))));
    //Total distance in meters (x-direction)
      float dx = fabs(d2 * DLON_D);

  //Calculate target angle for base servo, in radians
  float horizontal_angle_R = 0;
  if (DLAT_D > 0) {
    if (DLON_D > 0) {
      horizontal_angle_R = atan(dy / dx);
    }
    else {
       horizontal_angle_R = M_PI - atan(dy / dx);
    }
  }
  else {
    if (DLON_D > 0) {
      horizontal_angle_R = 2*M_PI - atan(dy / dx);
    }
    else {
       horizontal_angle_R = M_PI + atan(dy / dx);
    }
  }
  float horizontal_angle_D =  horizontal_angle_R * (180/M_PI); //Convert from radians to degrees

  //Calculate euclidean distance between ground station and target
  float E_distance_in_meters = sqrt(pow(dx, 2) + pow(dy, 2));
  //Calculate target angle for tilt servo, in radians
  float elevation_angle_R = atan((Talt - GSalt) / E_distance_in_meters);
  float elevation_angle_D = elevation_angle_R * (180/M_PI) //Convert from radians to degrees
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/mavros/global_position/global", 1000, callback);

  ros::spin();

  return 0;
}