#include "Slam.h"
#include "ScanToCloud.h"
#include <vector>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include "flirt/utils/SimpleMinMaxPeakFinder.h"


using namespace message_filters;
using namespace boost::numeric::ublas;

Slam::Slam(ros::NodeHandle node){ 
    n = node;
    
    zero_matrix<double> Q_zero(3,3);
    Q= Q_zero;
    Q.insert_element(0,0,0.3);
    
    vector<double> v(3);
    result.first = v;    
    
    zero_matrix<double> m(3,3);
    result.second = m;
}

void Slam::initNode(){

    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> subscriberOdom(n, "robot_pose_ekf/odom_combined", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud> subscriberCloud(n, "/cloud", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> subscriberLaser(n, "/base_scan", 1);
    
    publisherResult = n.advertise<geometry_msgs::PointStamped>("pose_corrected", 50);
    
    typedef sync_policies::ExactTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::PointCloud, sensor_msgs::LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriberOdom, subscriberCloud, subscriberLaser);
    sync.registerCallback(boost::bind(&Slam::slamCallback, this, _1, _2, _3));
    
}


void Slam::ekfSlam(geometry_msgs::PoseWithCovarianceStamped odom, sensor_msgs::LaserScan scan){
    
    SimpleMinMaxPeakFinder peakFinder;    
    LaserReading lr = makeLaserReading(odom, scan);
    ShapeContextGenerator desc(0,1500,1500,180);    
    std::vector<InterestPoint*> featuresCurrent;    
    RangeDetector rd(&peakFinder);
    
    double alpha = 0.3;
    //TODO Inicializar o RESULT!!!!!!!!!!!!
    
    result.first[0] = odom.pose.pose.position.x;
    result.first[1] = odom.pose.pose.position.y;
    
    btQuaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    double yaw,pitch,roll;
    btMatrix3x3(q).getEulerYPR(yaw,pitch,roll);
    
    result.first[2] = yaw;
    
    result.second(0,0) = odom.pose.covariance[0];
    result.second(0,1) = odom.pose.covariance[1];
    result.second(0,2) = odom.pose.covariance[5];
    result.second(1,0) = odom.pose.covariance[6];
    result.second(1,1) = odom.pose.covariance[7];
    result.second(1,2) = odom.pose.covariance[11];
    result.second(2,0) = odom.pose.covariance[30];
    result.second(2,1) = odom.pose.covariance[31];
    result.second(2,2) = odom.pose.covariance[35];
    
    
    rd.detect(lr, featuresCurrent);
    for(std::vector<InterestPoint*>::iterator it=featuresCurrent.begin(); it != featuresCurrent.end(); it++){
        (*it)->setDescriptor(desc.describe((**it), lr));
    }
    
    std::vector<InterestPoint*> featuresNew;
    
    result.first.resize(result.first.size() + 3);
    for(int i=1; i<=3; i++)
        result.first(result.first.size()-i) = 0;
    
    result.second.resize(result.second.size1() + 3, result.second.size2() + 3);
    for(int i=1; i<=3; i++){
        for(int j=1; j<=result.second.size2(); j++){
            result.second(result.second.size1()-i, result.second.size2()-j) = 0;
            result.second(result.second.size1()-j, result.second.size2()-i) = 0;
        }
    }
            
       
    for(std::vector<InterestPoint*>::iterator it2=featuresCurrent.begin(); it2 != featuresCurrent.end(); it2++){   
    
        int counter = 1;
        int counter_real;
        double pi;   
        vector<double> z_diff_real(3);
        matrix<double> H_real(3,3+(features.size()+1));
        matrix<double> Psi_real(3,3);
        features.push_back(*it2);
        
        result.first.resize(result.first.size() + 3);
        for(int i=1; i<=3; i++)
            result.first(result.first.size()-i) = 0;
    
        result.second.resize(result.second.size1() + 3, result.second.size2() + 3);
        for(int i=1; i<=3; i++){
            for(int j=1; j<=result.second.size2(); j++){
                result.second(result.second.size1()-i, result.second.size2()-j) = 0;
                result.second(result.second.size1()-j, result.second.size2()-i) = 0;
            }
        }
         
        
        for(std::vector<InterestPoint*>::iterator it=features.begin(); it != features.end(); it++){
        
            vector<double> delta_curr(2);
            delta_curr[0] = (*it2)->getPosition().x - odom.pose.pose.position.x;
            delta_curr[1] = (*it2)->getPosition().y - odom.pose.pose.position.y;     
            
            double q = inner_prod(delta_curr, delta_curr);
            
            vector<double> z(2);
            z[0] = sqrt(q);
            z[1] = atan2(delta_curr[1],delta_curr[0]) - yaw;                 
        
        
        
            vector<double> delta(2);
            delta[0] = (*it)->getPosition().x - odom.pose.pose.position.x;
            delta[1] = (*it)->getPosition().y - odom.pose.pose.position.y;
            
            q = inner_prod(delta, delta);
            
            vector<double> z_hat(2);
            z_hat[0] = sqrt(q);
            z_hat[1] = atan2(delta[1],delta[0]) - yaw;            
            
            
            
            vector<double> z_diff(3);
            z_diff[0] = z[0]-z_hat[0];
            z_diff[1] = z[1]-z_hat[1];
            z_diff[2] = (*it2)->getDescriptor()->distance((*it)->getDescriptor());
            
            matrix<double> F;           
            zero_matrix<double> F_zero(6, 3*(features.size()+1));
            F = F_zero;
                
            F.insert_element(0,0,1);
            F.insert_element(1,1,1);
            F.insert_element(2,2,1);
            F.insert_element(3*counter,4,1);
            F.insert_element(3*counter+1,5,1);
            F.insert_element(3*counter+2,6,1);
            
            zero_matrix<double> H_zero(3,6);
            matrix<double> H_temp(3,6);        
            H_temp = H_zero;
            
            H_temp.insert_element(0,0,-sqrt(q)*delta(0));       
            H_temp.insert_element(0,1,-sqrt(q)*delta(1));
            H_temp.insert_element(0,3,sqrt(q)*delta(0));
            H_temp.insert_element(0,4,sqrt(q)*delta(1));
            H_temp.insert_element(1,0,delta(1));
            H_temp.insert_element(1,1,-delta(0));
            H_temp.insert_element(1,2,-1);
            H_temp.insert_element(1,3,-delta(1));
            H_temp.insert_element(1,4,delta(0));
            H_temp.insert_element(2,5,1);
            
            matrix<double> H(3,3*(features.size()+1));
            H = (1/q)*prod(H_temp,trans(F));
            
            matrix<double> temp_m = prod(H, result.second);
            matrix<double> Psi = prod(temp_m, trans(H)) + Q;
            
            vector<double> temp = prod(z_diff,inverse(Psi));
            matrix<double> pi_temp = outer_prod(temp,z_diff);
            double pi_curr;
            
            if(counter == features.size()) pi_curr = alpha;
            else pi_curr = pi_temp(0,0);
            
            if(pi_curr < pi || counter==1){
                pi = pi_curr;
                H_real = H;
                Psi_real = Psi;
                z_diff_real = z_diff;
                counter_real = counter;
            }
            
        }   

        result.first.resize(result.first.size() + 3);
        
        matrix<double> temp_m2 = prod(result.second, trans(H_real));
        matrix<double> K = prod(temp_m2, inverse(Psi_real));
        
        result.first = result.first + prod(K,z_diff_real);
        result.second = prod((identity_matrix<double>(result.first.size(), 3) - prod(K,H_real)),result.second);
        
        if(counter_real != features.size()){
            features.pop_back();
            result.first.resize(result.first.size() - 3);
            result.second.resize(result.second.size1() - 3, result.second.size2() - 3);            
        }
        
        int i=0;
        for(std::vector<InterestPoint*>::iterator it=features.begin(); it != features.end(); it++){
            OrientedPoint2D point(result.first[3+3*i], result.first[3+3*i+1], 0);
            (*it)->setPosition(point);
            i++;
        }        
        
    }  
    
    features.insert(features.end(), featuresNew.begin(), featuresNew.end());
        
}


LaserReading Slam::makeLaserReading(geometry_msgs::PoseWithCovarianceStamped odom, sensor_msgs::LaserScan scan){
    
    std::vector<double> scanReading(scan.intensities.begin(),scan.intensities.end());
    std::vector<double> scanBearing(scan.ranges.begin(), scan.ranges.end());
    
    LaserReading lr(scanReading, scanBearing);
    
    btQuaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    double yaw,pitch,roll;
    btMatrix3x3(q).getEulerYPR(yaw,pitch,roll);
    
    OrientedPoint2D point(odom.pose.pose.position.x, odom.pose.pose.position.y, yaw);
    
    lr.setLaserPose(point);
    
    return lr;        
        
}


matrix<double> Slam::inverse(const matrix<double>& input)
{
	typedef permutation_matrix<std::size_t> pmatrix;

    matrix<double> inverse;

	// create a working copy of the input
	matrix<double> A(input);

	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = lu_factorize(A, pm);	
	
	if (res != 0)
		return inverse;

	// create identity matrix of "inverse"
	inverse.assign(identity_matrix<double> (A.size1()));

	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);

	return inverse;
}




void Slam::slamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odomMsg, 
                        const sensor_msgs::PointCloud::ConstPtr& cloudMsg,
                        const sensor_msgs::LaserScan::ConstPtr& laserMsg){

    ekfSlam(*odomMsg, *laserMsg);
    
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = cloudMsg->header.stamp;
    
    geometry_msgs::Point p;
    p.x = result.first[0];
    p.y = result.first[1];
    p.z = 0;
    pose.pose.position = p;
    
    tf::Quaternion q(cos(result.first[2]/2), 0, 0, sin(result.first[2]/2));
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    publisherResult.publish(pose);   
    
}






