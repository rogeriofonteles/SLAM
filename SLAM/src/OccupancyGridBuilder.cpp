#include "OccupancyGridBuilder.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace message_filters;

OccupancyGridBuilder::OccupancyGridBuilder(ros::NodeHandle node){n = node;}


void OccupancyGridBuilder::initNode(){

    message_filters::Subscriber<geometry_msgs::PointStamped> subscriberPose(n, "pose_corrected", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud> subscriberCloud(n, "cloud", 1);
    
    publisherMap = n.advertise<nav_msgs::OccupancyGrid>("pose_corrected", 50);
    
    typedef sync_policies::ExactTime<geometry_msgs::PointStamped, sensor_msgs::PointCloud> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriberPose, subscriberCloud);
    sync.registerCallback(boost::bind(&OccupancyGridBuilder::occCallback, this, _1, _2));
    
    occGrid = new OccupancyGrid(0,0,0,100,100,100,0.1,0.1,0.1);
    
}


void OccupancyGridBuilder::occCallback(const geometry_msgs::PointStamped::ConstPtr& poseMsg, const sensor_msgs::PointCloud::ConstPtr& cloudMsg){

    sensor_msgs::PointCloud tempCloud = *cloudMsg;

    for (size_t i = 0; i < tempCloud.points.size(); i++){
        tempCloud.points[i].x += poseMsg->point.x;
        tempCloud.points[i].y += poseMsg->point.y;
        tempCloud.points[i].z += poseMsg->point.z;
    }
    
    occGrid->fillOccupancyGrid(tempCloud);
    
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
    map.data.assign(data, data+sizeof(data));
    
    publisherMap.publish(map);       
    
}





