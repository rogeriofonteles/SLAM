#include "RansacMultiFeatureSetMatcher.h"

#include <boost/random.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <sys/time.h>


RansacMultiFeatureSetMatcher::RansacMultiFeatureSetMatcher(double acceptanceThreshold, double successProbability, double inlierProbability, double distanceThreshold, double rigidityThreshold, bool adaptive):
    RansacFeatureSetMatcher(acceptanceThreshold, successProbability, inlierProbability, distanceThreshold, rigidityThreshold, adaptive)
{

}

double RansacMultiFeatureSetMatcher::matchSets(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, OrientedPoint2D& transformation,
					  std::vector< std::pair<InterestPoint *, InterestPoint *> > &correspondences) const
{
    correspondences.clear();
    unsigned int iterations = m_adaptive ? 1e17 : ceil(log(1. - m_successProbability)/log(1. - m_inlierProbability * m_inlierProbability));
    
    // Compute possible correspondences based on thresholding
    std::vector< std::pair<InterestPoint *, InterestPoint *> > possibleCorrespondences;
    for(unsigned int i = 0; i < data.size(); i++){
	for(unsigned int j = 0; j < reference.size(); j++){
	    double distance = data[i]->getDescriptor()->distance(reference[j]->getDescriptor());
	    if(distance < m_distanceThreshold){
		possibleCorrespondences.push_back(std::make_pair(data[i], reference[j]));
	    }
	}
    }
    
    // Check if there are enough absolute matches 
    if(possibleCorrespondences.size() < 2){  
	return 1e17;
    }
    
    // Check if there are enough matches compared to the inlier probability (FIXME maybe is better to adjust the inlier probability)
    if(double(possibleCorrespondences.size()) * m_inlierProbability < 2){  
	return 1e17;
    }
    
    boost::mt19937 rng;
    boost::uniform_smallint<int> generator(0, possibleCorrespondences.size() - 1);
    
    // Main loop
    double minimumScore = 1e17;
    for(unsigned int i = 0; i < iterations; i++){
	unsigned int first = generator(rng);
	unsigned int second = generator(rng);
	while(second == first) second = generator(rng); // avoid useless samples
	std::pair< std::pair<InterestPoint *, InterestPoint *>, std::pair<InterestPoint *, InterestPoint *> > minimumSampleSet(possibleCorrespondences[first], possibleCorrespondences[second]);
	
	// Test rigidity
	const Point2D& diffFirst = possibleCorrespondences[first].first->getPosition() - possibleCorrespondences[second].first->getPosition();
	const Point2D& diffSecond = possibleCorrespondences[first].second->getPosition() - possibleCorrespondences[second].second->getPosition();
	double distanceFirst = diffFirst * diffFirst;
	double distanceSecond = diffSecond * diffSecond;
	if((distanceFirst - distanceSecond)*(distanceFirst - distanceSecond)/(8*(distanceFirst + distanceSecond	)) > m_rigidityThreshold){
	    continue;
	}
	
	// Compute hypothesis
	std::vector< std::pair<InterestPoint *, InterestPoint *> > inlierSet;
	OrientedPoint2D hypothesis = generateHypothesis(minimumSampleSet);
	
	// Verify hypothesis
	double score = verifyHypothesis(reference, data, hypothesis, inlierSet);
	if(score < minimumScore){
	    minimumScore = score;
	    transformation = hypothesis;
	    correspondences = inlierSet;
	    
	    // Adapt the number of iterations
	    if (m_adaptive){
		double inlierProbability = double(correspondences.size())/double(data.size());
		iterations = ceil(log(1. - m_successProbability)/log(1. - inlierProbability * inlierProbability));
	    }
	}
    }
    std::vector<std::pair<Point2D, Point2D> > pointCorrespondences(correspondences.size());
    for(unsigned int i = 0; i < correspondences.size(); i++){
	pointCorrespondences[i] = std::make_pair(correspondences[i].first->getPosition(), correspondences[i].second->getPosition());
    }
    compute2DPose(pointCorrespondences, transformation);

    return verifyHypothesis(reference, data, transformation, correspondences);
}
