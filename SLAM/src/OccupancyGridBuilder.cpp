#include "OccupancyGridBuilder.h"
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>


OccupancyGridBuilder::OccupancyGridBuilder(ros::NodeHandle node){
	n = node;
	occGrid = new OccupancyGrid(0,0,0,10,10,1,0.01,0.01,1);
}


void OccupancyGridBuilder::initNode(){

    subscriberPose = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(n, "pose_corrected", 1);
    subscriberCloud = new message_filters::Subscriber<sensor_msgs::PointCloud2>(n, "cloud", 1);
    
    publisherMap = n.advertise<nav_msgs::OccupancyGrid>("map", 50);    
    
    sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *subscriberPose, *subscriberCloud);
    sync->registerCallback(boost::bind(&OccupancyGridBuilder::occCallback, this, _1, _2));   
}


void OccupancyGridBuilder::occCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg, const sensor_msgs::PointCloud2::ConstPtr& cloudMsg){

	ROS_INFO("Occupancy Grid Callback");

    sensor_msgs::PointCloud2 cloudTransf2ROS;
    sensor_msgs::PointCloud cloudTransfROS, cloudTemp;
    
    sensor_msgs::convertPointCloud2ToPointCloud(*cloudMsg, cloudTemp);
    
    Eigen::Quaternionf q(poseMsg->pose.pose.orientation.x, poseMsg->pose.pose.orientation.y, poseMsg->pose.pose.orientation.z, poseMsg->pose.pose.orientation.w);
    
    Eigen::Vector3f t(poseMsg->pose.pose.position.x, poseMsg->pose.pose.position.y, 0.0);

    pcl::PointCloud<pcl::PointXYZ> cloud, cloudTransf;
    pcl::fromROSMsg (*cloudMsg, cloud);

	pcl::transformPointCloud(cloud, cloudTransf, t, q);
	
	pcl::toROSMsg(cloudTransf, cloudTransf2ROS);
	sensor_msgs::convertPointCloud2ToPointCloud(cloudTransf2ROS, cloudTransfROS);
    
    std::cout << cloudTemp.points[0] << " " << cloudTransfROS.points[0] << std::endl;
    
    occGrid->fillOccupancyGrid(cloudTransfROS);
    
    nav_msgs::OccupancyGrid map;    
    map.info.map_load_time = cloudMsg->header.stamp;
    map.info.resolution = 0.1;
    map.info.height = occGrid->nY();
    map.info.width = occGrid->nX();
    
    geometry_msgs::Pose pose;
    pose.position.x = 0; pose.position.y = 0; pose.position.z = 0;
    pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
    
    map.info.origin = pose;
    int8_t *data = occGrid->getData();
    int nCells = occGrid->nX() * occGrid->nY() * occGrid->nZ(); 
    map.data.resize(nCells);
    for (int i=0; i<nCells; i++) map.data[i] = data[i];
    
    publisherMap.publish(map);       
    
    ROS_INFO("Fim Occupancy Grid Callback");
    
}





