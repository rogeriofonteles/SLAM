#include "Slam.h"
#include "ScanToCloud.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/numeric/ublas/vector.hpp>
#include "flirt/utils/SimpleMinMaxPeakFinder.h"


using namespace message_filters;
using namespace boost::numeric::ublas;

Slam::Slam(ros::NodeHandle node){ 
    n = node;
    Q(0,0) = 0.3;    
}

void Slam::initNode(){

    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> subscriberOdom(n, "robot_pose_ekf/odom", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud> subscriberCloud(n, "/cloud", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> subscriberLaser(n, "/base_scan", 1);
    
    publisherResult = n.advertise<geometry_msgs::PointStamped>("pose_corrected", 50);
    
    typedef sync_policies::ExactTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::PointCloud> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriberOdom, subscriberLaser);
    sync.registerCallback(boost::bind(&Slam::slamCallback, this, _1, _2));
    
}


void Slam::ekfSlam(geometry_msgs::PoseWithCovarianceStamped odom, sensor_msgs::LaserScan scan){
    
    SimpleMinMaxPeakFinder peakFinder;    
    LaserReading lr = makeLaserReading(odom, scan);
    ShapeContext desc;    
    std::vector<InterestPoints*> featuresCurrent;    
    RangeDetector rd(&peakFinder);
    
    
    rd.detect(featuresCurrent, lr);
    for(it=featuresCurrent.begin(); it != featuresCurrent.end(); it++){
        desc.describe(it*, lr)
    }
    
    vector<InterestPoint*> featuresNew;
       
    for(it2=featuresCurrent.begin(); it2 != featuresCurrent.end(); it2++){   
    
        int counter = 1;
        int counter_real;
        double pi;   
        vector<double> z_diff_real(3);
        matrix<double> H_real(3,3+(features.size()+1));
        matrix<double> Psi_real(3,3);
        
        for(it=features.begin(); it != features.end(); it++){
        
            vector<double> delta_curr(2);
            delta_curr(0) = it2->getPosition().x - odom.pose.pose.position.x;
            delta_curr(1) = it2->getPosition().y - odom.pose.pose.position.y;     
            
            double q = inner_prod(delta_curr, delta_curr);
            
            vector<double> z(2);
            z(0) = sqrt(q);
            z(1) = atan2(delta_curr(1),delta_curr(0)) - odom.pose.pose.orientation.w;                 
        
        
        
            vector<double> delta(2);
            delta(0) = it->getPosition().x - odom.pose.pose.position.x;
            delta(1) = it->getPosition().y - odom.pose.pose.position.y;
            
            q = inner_prod(delta, delta);
            
            vector<double> z_hat(2);
            z_hat(0) = sqrt(q);
            z_hat(1) = atan2(delta(1),delta(0)) - odom.pose.pose.orientation.w;            
            
            
            
            vector<double> z_diff(3);
            z_diff(0) = z(0)-z_hat(0);
            z_diff(1) = z(1)-z_hat(1);
            z_diff(2) = it2->getDescriptor()->distance(it->getDescriptor());
            
            zero_matrix<double> F(6,3+(features.size()+1));
            F(0,0) = F(1,1) = F(2,2) = 1;
            F(3*counter,4) = F(3*counter+1,5) = F(3*counter+2,6) = 1;
            
            zero_matrix<double> H_temp(3,6);        
            H_temp(0,0) = -sqrt(q)*delta(0);       
            H_temp(0,1) = -sqrt(q)*delta(1);
            H_temp(0,3) = sqrt(q)*delta(0);
            H_temp(0,4) = sqrt(q)*delta(1);
            H_temp(1,0) = delta(1);
            H_temp(1,1) = -delta(0);
            H_temp(1,2) = -1;
            H_temp(1,3) = -delta(1);
            H_temp(1,4) = delta(0);
            H_temp(2,5) = 1;
            
            matrix<double> H(3,3+(features.size()+1));
            H = (1/q)*H_temp*trans(F);
            
            matrix<double> Psi(3,3);
            
            Psi = (H*Gaussian.second)*trans(H) + Q;
            
            double pi_curr = (trans(z_diff)*inverse(Psi))*z_diff;
            
            if(pi_curr < pi || counter==1){
                pi = pi_curr;
                H_real = H;
                Psi_real = Psi;
                z_diff_real = z_diff;
                counter_real = counter;
            }           
            
        }   
        
        if(counter_real >= result.first.size()){
           //TODO aumenta u, Sigma, H     
           featuresNew.push_back(it2);
           result.first.resize(result.first.size() + 3);
           result.second.resize(result.second.size1() + 3, result.second.size2() + 3);
           H_real.resize(H_real.size1() + 3, H_real.size2() + 3);
        }
        
        matrix<double> K(result.first.size(),3);
        //TODO H_real_ext
        K = (result.second*trans(H_real))*inverse(Psi_real);
        
        result.first = result.first + K*z_diff_real;
        result.second = (identity_matrix<double>(result.first.size(), 3) - K*H_real)*result.second;
    }  
        
}

LaserReading Slam::makeLaserReading(geometry_msgs::PoseWithCovarianceStamped odom, sensor_msgs::LaserScan scan){
    
    std::vector<float> scanReading(scan.intensities, scan.intensities+sizeof(scan.intensities));
    std::vector<float> scanBearing(scan.ranges, scan.range+sizeof(scan.range));
    
    LaserReading lr(scanReading, scanBearing);
    
    //TODO theta para rotacao da nuvem d pontos do sensor
    
    OrientedPoint2D point(odom.pose.pose.position.x, odom.pose.pose.position.y, );
    
    lr.setLaserPose(point);
    
    return lr;        
        
}


matrix<T> Slam::inverse(const matrix<T>& input)
{
	typedef permutation_matrix<std::size_t> pmatrix;

    matrix<T> inverse;

	// create a working copy of the input
	matrix<T> A(input);

	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = lu_factorize(A, pm);
	if (res != 0)
		return NULL;

	// create identity matrix of "inverse"
	inverse.assign(identity_matrix<T> (A.size1()));

	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);

	return inverse;
}




void Slam::slamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odomMsg, 
                        const sensor_msgs::PointCloud::ConstPtr& cloudMsg,
                        const sensor_msgs::LaserScan::ConstPtr& laserMsg){

    //slam.ekfSlam(*odomMsg, *laserMsg);
    
    geometry_msgs::PointStamped pose;
    pose.header.stamp = cloudMsg->header.stamp;
    pose.point = odomMsg->pose.pose.position;
    
    publisherResult.publisH_temp(pose);   
    
}





