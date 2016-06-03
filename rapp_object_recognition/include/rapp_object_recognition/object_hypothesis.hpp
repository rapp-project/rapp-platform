#ifndef OBJECT_HYPOTHESIS
#define OBJECT_HYPOTHESIS

#include <opencv2/core/core.hpp>

#include "rapp_object_recognition/object_model.hpp"

class ObjectHypothesis {
public:
	/// Object center in query image
	cv::Point2f center;
	
	/// Objects bounding box corners in query image
	std::vector<cv::Point2f> corners;
	
	/// Hypothesis score
	double score;
	
	/// Hypothesis validity
	bool valid;
	
	/// Model of detected object
	ObjectModel * model;
};

#endif /* OBJECT_HYPOTHESIS */
