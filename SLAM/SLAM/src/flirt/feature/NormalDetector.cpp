#include "NormalDetector.h"

#include <iostream>


NormalDetector::NormalDetector(const PeakFinder* peak, unsigned int scales, double sigma, double step, unsigned int window, SmoothingFilterFamily filterType):
    MultiScaleDetector(peak, scales, sigma, step, filterType),
    m_windowSize(window)
{
//     computeDifferentialBank();
}

// void NormalDetector::computeDifferentialBank(){
//     m_differentialBank.resize(m_scaleNumber, std::vector<double>(3));
//     for(unsigned int i = 0; i < m_differentialBank.size(); i++){
// 	m_differentialBank[i][0] = m_scales[i]*1; 
// 	m_differentialBank[i][1] = -m_scales[i]*2; 
// 	m_differentialBank[i][2] = m_scales[i]*1;
//     }
// }

void NormalDetector::computeSignal(const LaserReading& reading, std::vector<double>& signal, std::vector<unsigned int>& maxRangeMapping) const{
//    const std::vector<Point2D>& points = reading.getCartesian();
    std::vector<double> ranges;
    ranges.reserve(reading.getRho().size());
    maxRangeMapping.reserve(reading.getRho().size());
    for(unsigned int i = 0; i < reading.getRho().size(); i++){
		if(reading.getRho()[i] < reading.getMaxRange()){ 
			ranges.push_back(reading.getRho()[i]);
			maxRangeMapping.push_back(i);
		} else if (m_useMaxRange){
			ranges.push_back(reading.getMaxRange());
			maxRangeMapping.push_back(i);
		}
    }


    int offsetRange = floor((int)m_filterBank[0].size()/2.0);
    const std::vector<double>& rangeData = convolve1D(ranges, m_filterBank[0], -offsetRange); 
    const std::vector<double>& phiData = reading.getPhi();
    std::vector<Point2D> points(rangeData.size());
    
    for(unsigned int i = 0; i < rangeData.size(); i++){
		if(rangeData[i]<reading.getMaxRange()){
			points[i].x = cos(phiData[maxRangeMapping[i]])*rangeData[i];
			points[i].y = sin(phiData[maxRangeMapping[i]])*rangeData[i];
		} else {
			points[i].x = cos(phiData[maxRangeMapping[i]])*reading.getMaxRange();
			points[i].y = sin(phiData[maxRangeMapping[i]])*reading.getMaxRange();
		}
    }

    signal.resize(points.size());
    unsigned int offset = floor((double)m_windowSize * 0.5);
    std::vector<Point2D>::const_iterator first = points.begin();
    std::vector<Point2D>::const_iterator last = first + m_windowSize;
    double oldangle = 0;
    for(unsigned int i = offset; i < signal.size() - offset; i++){
		LineParameters param = computeNormals(std::vector<Point2D>(first,last));
		signal[i] = normAngle(param.alpha, oldangle - M_PI);
		oldangle = signal[i];
		first++; last++;
    }
    for(unsigned int i = 0; i < offset; i++){
		signal[i] = signal[offset];
    }
    for(unsigned int i = signal.size() - offset; i < signal.size(); i++){
		signal[i] = signal[signal.size() - offset - 1];
    }
}

unsigned int NormalDetector::computeInterestPoints(const LaserReading& reading, const std::vector<double>& signal, std::vector<InterestPoint*>& point, 
						   std::vector< std::vector<unsigned int> >& indexes, std::vector<unsigned int>& maxRangeMapping) const
{
    point.clear();
    point.reserve(reading.getRho().size());
    const std::vector<Point2D>& worldPoints = reading.getWorldCartesian();
    for(unsigned int i = 0; i < indexes.size(); i++){
		for(unsigned int j = 0; j < indexes[i].size(); j++){
			OrientedPoint2D pose;
			unsigned int pointIndex = maxRangeMapping[indexes[i][j]];
		
			// Reomoving the detection in the background and pushing it to the foreground
			double rangeBefore = (pointIndex > 1)? reading.getRho()[pointIndex - 1] : reading.getMaxRange();
			double rangeAfter = (pointIndex < worldPoints.size() - 1)? reading.getRho()[pointIndex + 1] : reading.getMaxRange();
			double rangeCurrent = reading.getRho()[pointIndex];
			if(rangeBefore < rangeAfter){
				if(rangeBefore < rangeCurrent){
					pointIndex = pointIndex - 1;
				}
			} else if(rangeAfter < rangeCurrent){
				pointIndex = pointIndex + 1;
			}
			
			// Removing max range reading
			if(reading.getRho()[pointIndex] >= reading.getMaxRange()){
				continue;
			}

			pose.x =  (reading.getWorldCartesian()[pointIndex]).x;
			pose.y =  (reading.getWorldCartesian()[pointIndex]).y;
			pose.theta = normAngle(signal[indexes[i][j]], -M_PI);
			
			bool exists = false;
			for(unsigned int k = 0; !exists && k < point.size(); k++){
				exists = exists || (fabs(pose.x - point[k]->getPosition().x) <= 0.2 &&  fabs(pose.y - point[k]->getPosition().y) <= 0.2);
			}
			if(exists) continue;

	    unsigned int first = indexes[i][j] - floor((int)m_filterBank[i].size()/2.0);
	    unsigned int last = indexes[i][j] + floor((int)m_filterBank[i].size()/2.0);
	    std::vector<Point2D> support(last - first + 1);
	    for(unsigned int p = 0; p < support.size(); p++) {
		support[p] = Point2D(worldPoints[maxRangeMapping[p + first]]);
	    }
	    
			double maxDistance = -1e20;
			for(unsigned int k = 0; k < support.size(); k++){
				double distance = sqrt((pose.x - support[k].x)*(pose.x - support[k].x) + (pose.y - support[k].y)*(pose.y - support[k].y));
				maxDistance = maxDistance < distance ? distance : maxDistance;
			}
			InterestPoint *interest = new InterestPoint(pose, maxDistance);
	// 	    InterestPoint *interest = new InterestPoint(pose, m_scales[i]);
			interest->setSupport(support);
			interest->setScaleLevel(i);
			point.push_back(interest);
		}
    }
    return point.size();
}

