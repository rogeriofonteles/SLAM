#include <ros/ros.h>
#include "Slam.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "slam");
  ros::NodeHandle n; //Listener
  
  Slam slam(n);
  slam.initNode();
  
  ros::spin();

  return 0;
}
