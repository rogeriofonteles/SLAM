#include <ros/ros.h>
#include "ScanToCloud.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");
    ScanToCloud filter;

    ros::spin();
    return 0;
}
