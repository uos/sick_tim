#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_tim3xx");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

  // TODO: init scanner

  while (ros::ok())
  {
    sensor_msgs::LaserScan msg;

    // TODO: read data, fill msg

    pub.publish(msg);

    ros::spinOnce(); // do we need this?
  }

  return 0;
}
