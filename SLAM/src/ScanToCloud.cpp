#include "ScanToCloud.h"

ScanToCloud::ScanToCloud(){

    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("scan", 100, &ScanToCloud::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(5.0));
    
}


void ScanToCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

    sensor_msgs::PointCloud cloud;
    tfListener_.waitForTransform("base_link", "base_scan", scan->header.stamp, ros::Duration(10.0) );
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
    
}

