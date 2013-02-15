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
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <utility>

using namespace boost::numeric::ublas;

typedef std::pair<vector<double>,matrix<double> > Gaussian;

class Slam{

    public:
    
        Slam(ros::NodeHandle);
        
        ros::NodeHandle n;
        ros::Publisher publisherResult;   
    
        message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> subscriberOdom;
        message_filters::Subscriber<sensor_msgs::PointCloud> subscriberCloud
        message_filters::Subscriber<sensor_msgs::LaserScan> subscriberLaser;
    
        void initNode();
    
        void slamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, const sensor_msgs::PointCloud::ConstPtr&);
        void ekfSlam(sensor_msgs::PointCloud, geometry_msgs::PoseWithCovarianceStamped);
        bool inverse(const matrix<T>&)
        
        
    private:
    
        sensor_msgs::PointCloud pastCloud;                 
        geometry_msgs::PoseWithCovarianceStamped pastOdom;
        
        std::vector<InterestPoint*> features;
        zero_matrix<double> Q;
        
        Gaussian result;
};

#endif
