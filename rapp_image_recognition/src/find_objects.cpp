#include <vector>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>

namespace FindObjects {


// ******************************** MODELS ********************************

// Vector of images constituting the consecutive models.
std::vector<cv::Mat> models_imgs;

/// Vector of keypoints of consecutive models.
std::vector<std::vector<cv::KeyPoint> > models_keypoints;

/// Vector of descriptors of consecutive models.
std::vector<cv::Mat> models_descriptors;

/// Vector of names of consecutive models.
std::vector<std::string> models_names;

// ****************************** HYPOTHESES ******************************

/// Vector containing names of recognized objects.
std::vector<std::string> recognized_names;

/// Vector containing centers of recognized objects (image coordinates).
std::vector<cv::Point2f> recognized_centers;

/// Vector containing quadruples of corners of recognized objects (image coordinates).
std::vector<std::vector<cv::Point2f> > recognized_corners;

/// Vector containint scores of recognized objects.
std::vector<double> recognized_scores;


// **************************** OTHER VARIABLES ****************************

using namespace cv;

/// Keypoint detector.
cv::OrbFeatureDetector detector;

/// Feature descriptor.
cv::OrbDescriptorExtractor extractor;

// Matcher.
cv::BFMatcher matcher(NORM_L2, true);


// ******************************* FUNCTIONS *******************************
bool loadImage(const std::string filename_, cv::Mat & image_) {
	try {
		// TODO: fix the case of invalid file.
	        image_ = imread( filename_ );
		return true;
	} catch (...) {
		ROS_WARN ("Could not load image from file %s", filename_.c_str());
		return false;
	}
}


bool extractFeatures(const cv::Mat image_, std::vector<KeyPoint> & keypoints_, cv::Mat & descriptors_) {
        cv::Mat gray_img;

	try {
		// Transform to grayscale - if requred.
		if (image_.channels() == 1)
			gray_img = image_;
		else 
			cvtColor(image_, gray_img, COLOR_BGR2GRAY);

		// Detect the keypoints.
		detector.detect( gray_img, keypoints_ );

		ROS_DEBUG("Detected %d keypoints", keypoints_.size());

		// Extract descriptors (feature vectors).
		extractor.compute( gray_img, keypoints_, descriptors_ );
		return true;
	} catch (...) {
		ROS_WARN ("Could not extract features from image");
		return false;
	}//: catch
}


void loadModels(std::vector<std::string> names_, std::vector<std::string> files_){

	cv::Mat model_img;

	// Clear database.
	models_imgs.clear();
	models_keypoints.clear();
	models_descriptors.clear();
	models_names.clear();

	// Load all models - read image, extract feature and add data to lists.
	for (int i=0; i<names_.size(); i++) {
		if ( loadImage(files_[i], model_img) ) {
			ROS_DEBUG("Size of loaded image (%d,%d)", model_img.size().width, model_img.size().height );

			std::vector<cv::KeyPoint> model_keypoints;
			cv::Mat model_descriptors;
			extractFeatures(model_img, model_keypoints, model_descriptors);

			// Add to "database".
			models_imgs.push_back(model_img);
			models_keypoints.push_back(model_keypoints);
			models_descriptors.push_back(model_descriptors);
			models_names.push_back(names_[i]);

			ROS_INFO("Successfull load of model %s from file %s", names_[i].c_str(), files_[i].c_str());
		}//: if
	}//: for
		

}


void storeObjectHypothesis(std::string name_, cv::Point2f center_, std::vector<cv::Point2f> corners_, double score_, unsigned int limit_) {

	// Special case: do not insert anything is smaller than one;)
	if (limit_<1)
		return;

	// Special case: insert first object hypothesis.
	if (recognized_names.size() == 0) {
		ROS_INFO("Adding first (0) object hypothesis");
		recognized_names.push_back(name_);
		recognized_centers.push_back(center_);
		recognized_corners.push_back(corners_);
		recognized_scores.push_back(score_);
		return;
	}//: if

	// Iterators.
	std::vector<std::string>::iterator names_it = recognized_names.begin();
	std::vector<cv::Point2f>::iterator centers_it = recognized_centers.begin();
	std::vector<std::vector<cv::Point2f> >::iterator corners_it= recognized_corners.begin();
	std::vector<double>::iterator scores_it= recognized_scores.begin();

	bool added = false;
	// Insert in proper order.
	for (; names_it != recognized_names.end(); names_it++, centers_it++, corners_it++, scores_it++) {
		if (*scores_it < score_){
			// Insert here! (i.e. before) - stop the iterator at this place.
			break;
		}//: if
	}//: for
	ROS_INFO("Adding next object hypothesis");
	recognized_names.insert(names_it, name_);
	recognized_centers.insert(centers_it, center_);
	recognized_corners.insert(corners_it, corners_);
	recognized_scores.insert(scores_it, score_);


	// Limit the size of vectors.
	if (recognized_names.size() > limit_){
		ROS_INFO("Removing last object hypothesis");
		recognized_names.pop_back();
		recognized_centers.pop_back();
		recognized_corners.pop_back();
		recognized_scores.pop_back();
	}//: if
}


int findObjects(std::string fname, std::vector<std::string> names, std::vector<std::string> files, unsigned int limit, 
                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores){

	ROS_INFO("Finding objects with the use of %d models", names.size());
  
	// Re-load the model - extract features from model.
	loadModels(names, files);
	if (models_imgs.size() ==0)
		return -1;

	// Tmp variables.
	std::vector<KeyPoint> scene_keypoints;
	cv::Mat scene_descriptors;
	std::vector< DMatch > matches;

	// Clear vectors!
	recognized_names.clear();
	recognized_centers.clear();
	recognized_corners.clear();
	recognized_scores.clear();

	// Load image containing the scene.
	cv::Mat scene_img;
	if (!loadImage(fname, scene_img))
		return -2;

	// Extract features from scene.
	extractFeatures(scene_img, scene_keypoints, scene_descriptors);
	ROS_INFO ("Scene features: %d", scene_keypoints.size());

	// Iterate - try to detect each model one by one.
	for (unsigned int m=0; m < models_imgs.size(); m++) {
		ROS_DEBUG ("Trying to recognize model (%d): %s", m, models_names[m].c_str());

		if ((models_keypoints[m]).size() == 0) {
			ROS_WARN ("Model %s not valid as it does not contain texture.", models_names[m].c_str());
			continue;
		}//: if

		ROS_DEBUG ("Model features: %d", models_keypoints[m].size());

		// Find matches.
		matcher.match( models_descriptors[m], scene_descriptors, matches );

		ROS_DEBUG ("Matches found: %d", matches.size());

		// Filtering.
		double max_dist = 0;
		double min_dist = 100;

		//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < matches.size(); i++ ) {
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}//: for

		ROS_DEBUG ("Max dist : %f", max_dist);
		ROS_DEBUG ("Min dist : %f", min_dist);

		//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
		std::vector< DMatch > good_matches;

		for( int i = 0; i < matches.size(); i++ ) {
			if( matches[i].distance < 3*min_dist )
				good_matches.push_back( matches[i]);
		}//: for

		ROS_DEBUG ("Good matches: %d", good_matches.size());

		// Localize the object
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		// Get the keypoints from the good matches.
		for( int i = 0; i < good_matches.size(); i++ ) {
		  obj.push_back( models_keypoints [m] [ good_matches[i].queryIdx ].pt );
		  scene.push_back( scene_keypoints [ good_matches[i].trainIdx ].pt );
		}//: for

		// Find homography between corresponding points.
		Mat H = findHomography( obj, scene, CV_RANSAC );

		// Get the corners from the detected "object hypothesis".
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cv::Point2f(0,0);
		obj_corners[1] = cv::Point2f( models_imgs[m].cols, 0 );
		obj_corners[2] = cv::Point2f( models_imgs[m].cols, models_imgs[m].rows );
		obj_corners[3] = cv::Point2f( 0, models_imgs[m].rows );
		std::vector<Point2f> hypobj_corners(4);

		// Transform corners with found homography.
		perspectiveTransform( obj_corners, hypobj_corners, H);
		
		// Verification: check resulting shape of object hypothesis.
		// Compute "center of mass".
		cv::Point2f center = (hypobj_corners[0] + hypobj_corners[1] + hypobj_corners[2] + hypobj_corners[3])*.25;
		std::vector<double> angles(4);
		cv::Point2f tmp;

		// Compute angles.
		for (int i=0; i<4; i++) {
			tmp = (hypobj_corners[i] - center);
			angles[i] = atan2(tmp.y,tmp.x);
		}//: if


		// Find smallest element.
		int imin = -1;
		double amin = 1000;
		for (int i=0; i<4; i++)
			if (amin > angles[i]) {
				amin = angles[i];
				imin = i;
			}//: if

		// Reorder table.
		for (int i=0; i<imin; i++) {
			angles.push_back (angles[0]);
			angles.erase(angles.begin());
		}//: for

		double score = (double)good_matches.size()/models_keypoints [m].size();
		// Check dependency between corners.
		if ((angles[0] < angles[1]) && (angles[1] < angles[2]) && (angles[2] < angles[3])) {
			// Order is ok.
			ROS_INFO ("Model (%d): keypoints= %d corrs= %d score= %f VALID", m, models_keypoints[m].size(), good_matches.size(), score);
			// Store the model in a list in proper order.
			storeObjectHypothesis(models_names[m], center, hypobj_corners, score, limit);

		} else {
			// Hypothesis not valid.
			ROS_INFO ("Model (%d): keypoints= %d corrs= %d score= %f REJECTED", m, models_keypoints[m].size(), good_matches.size(), score);
		}//: else
	}//: for
	
	/*	Mat img_object = scene_img.clone();
		if (recognized_names.size() == 0) {
			CLOG(LWARNING)<< "None of the models was not properly recognized in the image";
		} else {
		
			for (int h=0; h<recognized_names.size(); h++) {
				// Draw the final object - as lines, with center and top left corner indicated.
				line( img_object, recognized_corners[h][0], recognized_corners[h][1], Scalar(0, 255, 0), 4 );
				line( img_object, recognized_corners[h][1], recognized_corners[h][2], Scalar(0, 255, 0), 4 );
				line( img_object, recognized_corners[h][2], recognized_corners[h][3], Scalar(0, 255, 0), 4 );
				line( img_object, recognized_corners[h][3], recognized_corners[h][0], Scalar(0, 255, 0), 4 );
				circle( img_object, recognized_centers[h], 2, Scalar(0, 255, 0), 4);
				circle( img_object, recognized_corners[h][0], 2, Scalar(255, 0, 0), 4);
				CLOG(LNOTICE)<< "Hypothesis (): model: "<< recognized_names[h]<< " score: "<< recognized_scores[h];
			}//: for
		}//: else*/

	// Copy recognized objects to output variables.

	for(int i=0; i<recognized_names.size(); i++){ 
		found_names.push_back(recognized_names[i]);
		geometry_msgs::Point pt;
		pt.x = recognized_centers[i].x;
		pt.y = recognized_centers[i].y;
		pt.z = 0.0;
		found_centers.push_back(pt);
		found_scores.push_back(recognized_scores[i]);
	}//: for
 
	return 0;
}

};
