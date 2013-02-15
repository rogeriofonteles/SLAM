#ifndef OCCUPANCY_H
#define OCCUPANCY_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/numeric/ublas/matrix.hpp>
#include "OccupancyGrid.h"
#include <utility>

using namespace boost::numeric::ublas;

typedef std::pair<vector<double>,matrix<double> > Gaussian;

class OccupancyGridBuilder{

    public:
    
        OccupancyGridBuilder(ros::NodeHandle);
        
        ros::NodeHandle n;
        ros::Publisher publisherMap;
    
        message_filters::Subscriber<geometry_msgs::PointStamped> subscriberPose;
        message_filters::Subscriber<sensor_msgs::PointCloud> subscriberCloud;
    
        void initNode();
    
        void occCallback(const geometry_msgs::PointStamped::ConstPtr&, const sensor_msgs::PointCloud::ConstPtr&);
        
        
    private:
        OccupancyGrid *occGrid;
};

#endif
