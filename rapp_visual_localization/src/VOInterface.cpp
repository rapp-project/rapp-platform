/// File VOInterface.cpp
/// Methods of class CVisOdom (visual navigation in a known room)
/// designed for interfacing the real-time navigation process
///  
/// Author: 
/// Last modification: 24-07-2016
/// %%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "rapp_visual_localization/VisOdom.hpp"

::cv::Mat CVisOdom::inObservationImage() // from robot's camera to CVisOdom
{
	::cv::Mat inImg;
	//
	// ...
	//

	return inImg;
}

bool CVisOdom::inMotionVector(int *dz, int *dx, int *dd) // from robot's odometry to CVisOdom
{
	//
	// ...
	//
	return false; // false= no boundary (wall at distance of 0.5 m) detected, true= wall detected 
}

void CVisOdom::outBelief(double *belief, int bestZ, int bestX, int bestD) // from CVisOdom to robot's navigation control
{
	//

}
