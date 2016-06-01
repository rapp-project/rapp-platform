#include <vector>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <boost/filesystem.hpp>

#include "rapp_object_recognition/find_objects.hpp"

namespace fs = boost::filesystem;

std::string expand_user(std::string path) {
	if (not path.empty() and path[0] == '~') {
		assert(path.size() == 1 or path[1] == '/');  // or other error handling
		char const* home = getenv("HOME");
		if (home or (home = getenv("USERPROFILE"))) {
			path.replace(0, 1, home);
		}
		else {
			char const *hdrive = getenv("HOMEDRIVE");
			char const *hpath = getenv("HOMEPATH");
			assert(hdrive);  // or other error handling
			assert(hpath);
			path.replace(0, 1, std::string(hdrive) + hpath);
		}
	}
	return path;
}

// ******************************* FUNCTIONS *******************************
bool FindObjects::loadImage(const std::string filename_, cv::Mat & image_) {
	try {
		image_ = imread( filename_ );
		return !image_.empty();
	} catch (...) {
		if (!silent_) ROS_WARN ("Could not load image from file %s", filename_.c_str());
		return false;
	}
}


bool FindObjects::extractFeatures(const cv::Mat image_, std::vector<KeyPoint> & keypoints_, cv::Mat & descriptors_) {
        cv::Mat gray_img;

	try {
		// Transform to grayscale - if requred.
		if (image_.channels() == 1)
			gray_img = image_;
		else 
			cvtColor(image_, gray_img, COLOR_BGR2GRAY);

		// Detect the keypoints.
		detector.detect( gray_img, keypoints_ );

		if (!silent_) ROS_INFO("Detected %d keypoints", (int)keypoints_.size());

		// Extract descriptors (feature vectors).
		extractor.compute( gray_img, keypoints_, descriptors_ );
		return true;
	} catch (...) {
		if (!silent_) ROS_WARN ("Could not extract features from image");
		return false;
	}//: catch
}

int FindObjects::learnObject(const std::string & user, const std::string & fname, const std::string & name) {
	cv::Mat model_img;
	
	if (!loadImage(fname, model_img)) {
		return -2;
	}
	
	if (!silent_) ROS_DEBUG("Size of loaded image (%d,%d)", model_img.size().width, model_img.size().height );

	std::vector<cv::KeyPoint> model_keypoints;
	cv::Mat model_descriptors;
	extractFeatures(model_img, model_keypoints, model_descriptors);

	// store model on disk
	std::string fs_path = expand_user("~/rapp_platform_files/") + user + "/models/";
	if (!fs::exists(fs_path)) {
		fs::create_directories(fs_path);
	}
	fs_path += name + ".yml";
	if (!silent_) ROS_INFO("Model file: %s", fs_path.c_str());
	FileStorage fs(fs_path, FileStorage::WRITE);
	write( fs, "keypoints", model_keypoints );
	write( fs, "descriptors", model_descriptors);
	write( fs, "fname", fname);
	fs.release();

	if (!silent_) ROS_INFO("Successfull creation of model %s from file %s", name.c_str(), fname.c_str());
	
	return 0;
}

bool FindObjects::clearModels(const std::string & user) {
	models_keypoints.clear();

	models_descriptors.clear();

	models_names.clear();
	
	return true;
}

bool FindObjects::loadModel(const std::string & user, const std::string & name) {
	std::vector<cv::KeyPoint> model_keypoints;
	cv::Mat model_descriptors;
	std::string fname;
	
	std::string fs_path = expand_user("~/rapp_platform_files/") + user + "/models/" + name + ".yml";
	FileStorage fs(fs_path, FileStorage::READ);
	if (!fs.isOpened()) return false;
	
	cv::FileNode kptFileNode = fs["keypoints"];
	cv::read( kptFileNode, model_keypoints );
	fs["descriptors"] >> model_descriptors;
	fs["fname"] >> fname;
	fs.release();
	
	if (!silent_) ROS_INFO("Loaded model: %s", name.c_str());
	
	cv::Mat img = cv::imread(fname, -1);
	
	models_imgs.push_back(img);
	models_names.push_back(name);
	models_descriptors.push_back(model_descriptors);
	models_keypoints.push_back(model_keypoints);
	
	return true;
}

std::map<std::string, bool> FindObjects::loadModels(const std::string & user, const std::vector<std::string> & names, int & result) {
	std::map<std::string, bool> ret;
	for (size_t i = 0; i < names.size(); ++i) {
		ret[names[i]] = loadModel(user, names[i]);
	}
	
	result = 0;
	return ret;
}

void FindObjects::loadModels(std::vector<std::string> names_, std::vector<std::string> files_) {

	cv::Mat model_img;

	// Clear database.
	models_imgs.clear();
	models_keypoints.clear();
	models_descriptors.clear();
	models_names.clear();

	// Load all models - read image, extract feature and add data to lists.
	for (int i=0; i<names_.size(); i++) {
		if ( loadImage(files_[i], model_img) ) {
			if (!silent_) ROS_DEBUG("Size of loaded image (%d,%d)", model_img.size().width, model_img.size().height );

			std::vector<cv::KeyPoint> model_keypoints;
			cv::Mat model_descriptors;
			extractFeatures(model_img, model_keypoints, model_descriptors);

			// Add to "database".
			models_imgs.push_back(model_img);
			models_keypoints.push_back(model_keypoints);
			models_descriptors.push_back(model_descriptors);
			models_names.push_back(names_[i]);

			if (!silent_) ROS_INFO("Successfull load of model %s from file %s", names_[i].c_str(), files_[i].c_str());
		}//: if
	}//: for
		

}


void FindObjects::storeObjectHypothesis(std::string name_, cv::Point2f center_, std::vector<cv::Point2f> corners_, double score_, unsigned int limit_) {

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
	if (!silent_) ROS_INFO("Adding next object hypothesis");
	recognized_names.insert(names_it, name_);
	recognized_centers.insert(centers_it, center_);
	recognized_corners.insert(corners_it, corners_);
	recognized_scores.insert(scores_it, score_);


	// Limit the size of vectors.
	if (recognized_names.size() > limit_){
		if (!silent_) ROS_INFO("Removing last object hypothesis");
		recognized_names.pop_back();
		recognized_centers.pop_back();
		recognized_corners.pop_back();
		recognized_scores.pop_back();
	}//: if
}


int FindObjects::findObjects(const std::string & user, const std::string & fname, unsigned int limit, 
                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores){


	if (models_names.empty()) {
		if (!silent_) ROS_WARN("No models loaded");
		return -1;
	} else {
		if (!silent_) ROS_INFO("Finding objects with the use of %d models", (int)models_names.size());
	}
  
  
  
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
	if (!silent_) ROS_INFO ("Scene features: %d", (int)scene_keypoints.size());

	// Iterate - try to detect each model one by one.
	for (unsigned int m=0; m < models_names.size(); m++) {
		if (!silent_) ROS_DEBUG ("Trying to recognize model (%d): %s", m, models_names[m].c_str());

		if ((models_keypoints[m]).size() == 0) {
			if (!silent_) ROS_WARN ("Model %s not valid as it does not contain texture.", models_names[m].c_str());
			continue;
		}//: if

		if (!silent_) ROS_INFO ("Model features: %d", (int)models_keypoints[m].size());

		// Find matches.
		matcher.match( models_descriptors[m], scene_descriptors, matches );

		if (!silent_) ROS_INFO ("Matches found: %d", (int)matches.size());

		// Filtering.
		double max_dist = 0;
		double min_dist = 100;

		//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < matches.size(); i++ ) {
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}//: for

		if (!silent_) ROS_INFO ("Max dist : %f", max_dist);
		if (!silent_) ROS_INFO ("Min dist : %f", min_dist);

		//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
		std::vector< DMatch > good_matches;

		for( int i = 0; i < matches.size(); i++ ) {
			if( matches[i].distance <= 10*min_dist+0.1 )
				good_matches.push_back( matches[i]);
		}//: for

		if (!silent_) ROS_INFO ("Good matches: %d", (int)good_matches.size());

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
			if (!silent_) ROS_INFO ("Model (%d): keypoints= %d corrs= %d score= %f VALID", m, (int)models_keypoints[m].size(), (int)good_matches.size(), score);
			// Store the model in a list in proper order.
			storeObjectHypothesis(models_names[m], center, hypobj_corners, score, limit);

		} else {
			// Hypothesis not valid.
			if (!silent_) ROS_INFO ("Model (%d: %s): keypoints= %d corrs= %d score= %f REJECTED", m, models_names[m].c_str(), (int)models_keypoints[m].size(), (int)good_matches.size(), score);
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
