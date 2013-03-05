#ifndef OCCUPANCY_H
#define OCCUPANCY_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include "OccupancyGrid.h"
#include <utility>

using namespace boost::numeric::ublas;
using namespace message_filters;

typedef std::pair<vector<double>,matrix<double> > Gaussian;
typedef sync_policies::ExactTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::PointCloud2> MySyncPolicy;

class OccupancyGridBuilder{

    public:
    
        OccupancyGridBuilder(ros::NodeHandle);
        
        ros::NodeHandle n;
        ros::Publisher publisherMap;
    
        message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *subscriberPose;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *subscriberCloud;
        
        Synchronizer<MySyncPolicy> *sync;
    
        void initNode();
    
        void occCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&);
        
        
    private:
        OccupancyGrid *occGrid;
};

#endif
