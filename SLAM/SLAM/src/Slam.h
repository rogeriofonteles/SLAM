#ifndef SLAM_H
#define SLAM_H

#include <ros/ros.h>
#include "flirt/sensors/LaserReading.h"
#include "flirt/feature/InterestPoint.h"
#include "flirt/feature/Descriptor.h"
#include "flirt/feature/InterestPoint.h"
#include "flirt/feature/ShapeContext.h"
#include "flirt/feature/RangeDetector.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <utility>

using namespace boost::numeric::ublas;
using namespace message_filters;

typedef std::pair<vector<double>,matrix<double> > Gaussian;
typedef sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> MySyncPolicy;

class Slam{

    public:
    
        Slam(ros::NodeHandle);
        
        ros::NodeHandle n;
        ros::Publisher publisherResult;   
    
        message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *subscriberOdom;
        message_filters::Subscriber<sensor_msgs::LaserScan> *subscriberLaser;
    
        void initNode();
    
        void slamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, const  sensor_msgs::LaserScan::ConstPtr&);
        void ekfSlam(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, const sensor_msgs::LaserScan::ConstPtr&);
        LaserReading makeLaserReading(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, const sensor_msgs::LaserScan::ConstPtr&);
        matrix<double> inverse(const matrix<double>&);
        
        Synchronizer<MySyncPolicy> *sync;
        
        //TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> *sync;
        
    private:        
        
        std::vector<InterestPoint*> features;
        matrix<double> Q;
        
        Gaussian result;
};

#endif
