/*****************************************************************
 *
 * This file is part of the FLIRTLib project
 *
 * FLIRTLib Copyright (c) 2010 Gian Diego Tipaldi and Kai O. Arras 
 *
 * This software is licensed under the "Creative Commons 
 * License (Attribution-NonCommercial-ShareAlike 3.0)" 
 * and is copyrighted by Gian Diego Tipaldi and Kai O. Arras 
 * 
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/3.0/
 * 
 * FLIRTLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/



#ifndef RANSACMULTIFEATURESETMATCHER_H_
#define RANSACMULTIFEATURESETMATCHER_H_

#include "RansacFeatureSetMatcher.h"
#include "../utils/PoseEstimation.h"
#include <vector>
#include <utility>

/**
 * Representation of the RANSAC algorithm for feature matching.
 * The class represents the RANSAC algorithm for matching two different feature sets. 
 * The matching result is an Euclidean transformation (rotation + translation) encoded as a point in \f$ \mathcal{SO}(2) \f$.
 * The algorithm uses the Threshold strategy to obtain the possible correspondences from the features' descriptors. 
 * The main difference with the Nearest Neighobour is the increased complexity (reduced inlier probability) and the increased accuracy
 * especially in environment with very repetitive structures.
 *
 * @author Gian Diego Tipaldi
 */

class RansacMultiFeatureSetMatcher: public RansacFeatureSetMatcher {
    public:
	/**
	 * Constructor. Constructs and initializes the RANSAC algorithm.
	 *
	 * @param acceptanceThreshold The maximum distance (in meters) for a point to be considered in the inlier set.
	 * @param successProbability The probability of finding a correct match if exists.
	 * @param inlierProbability The probability of a generic correspondence to be an inlier.
	 * @param distanceThreshold The maximum distance (dimensionless) for two descriptors to be considered as a valid match. This threshold depends on the actual distance employed.
	 * @param rigidityThreshold The maximum value (in meters) of difference between the relative distance of two interest points. This implements a rigidity check in the RANSAC hypothesis generation.
	 * @param adaptive The flag to set the adaptive strategy to compute the number of RANSAC iterations (EXPERIMENTAL!!!).
	 */
	RansacMultiFeatureSetMatcher(double acceptanceThreshold, double successProbability, double inlierProbability, double distanceThreshold, double rigidityThreshold, bool adaptive = false);
	
	virtual double matchSets(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, OrientedPoint2D& transformation,
				 std::vector< std::pair<InterestPoint *, InterestPoint *> > &correspondences) const;

	/** Default destructor. */
	virtual ~RansacMultiFeatureSetMatcher() { }
};

#endif
