#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <boost/filesystem.hpp>

#include "rapp_object_recognition/find_objects.hpp"

//#define DEBUG_IMAGES
#define STRIP_ALL_ROSOUT

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

FindObjects::FindObjects(bool silent) { 
	detector = cv::FeatureDetector::create("ORB");
	extractor = cv::DescriptorExtractor::create("ORB");
	grid_detector = new cv::GridAdaptedFeatureDetector(detector, 4000, 4, 6);
	matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	silent_ = silent;
}


bool FindObjects::loadImage(const std::string & filename_, cv::Mat & image_) {
	try {
		image_ = imread( filename_ );
		return !image_.empty();
	} catch (...) {
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_WARN ("Could not load image from file %s", filename_.c_str());
#endif
		return false;
	}
}


bool FindObjects::extractFeatures(const cv::Mat image_, std::vector<KeyPoint> & keypoints_, cv::Mat & descriptors_, bool grid, cv::Mat mask_) {
	cv::Mat gray_img;

	try {
		// Transform to grayscale - if requred.
		if (image_.channels() == 1)
			gray_img = image_;
		else 
			cvtColor(image_, gray_img, COLOR_BGR2GRAY);

		// Detect the keypoints.
		detector->detect( gray_img, keypoints_, mask_ );
		if (grid) {
			std::vector<KeyPoint> tmp;
			grid_detector->detect( gray_img, tmp, mask_ );
			keypoints_.insert(keypoints_.end(), tmp.begin(), tmp.end());
		}
		
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO("Detected %d keypoints", (int)keypoints_.size());
#endif

		// Extract descriptors (feature vectors).
		extractor->compute( gray_img, keypoints_, descriptors_ );
		return true;
	} catch (...) {
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_WARN ("Could not extract features from image");
#endif
		return false;
	}//: catch
}

int FindObjects::learnObject(const std::string & user, const std::string & fname, const std::string & name) {
	ObjectModel model;
	
	if (!loadImage(fname, model.image)) {
		return -2;
	}

#ifndef STRIP_ALL_ROSOUT
	if (!silent_) ROS_DEBUG("Size of loaded image (%d,%d)", model.image.size().width, model.image.size().height );
#endif

	extractFeatures(model.image, model.keypoints, model.descriptors);

	// store model on disk
	std::string fs_path = expand_user("~/rapp_platform_files/") + user + "/models/";
	if (!fs::exists(fs_path)) {
		fs::create_directories(fs_path);
	}
	
	model.save(fs_path, name);
	
#ifndef STRIP_ALL_ROSOUT
	if (!silent_) ROS_INFO("Successfull creation of model %s from file %s", name.c_str(), fname.c_str());
#endif

	return 0;
}

bool FindObjects::clearModels(const std::string & user) {
	models.clear();	
	return true;
}

bool FindObjects::loadModel(const std::string & user, const std::string & name) {
	std::vector<cv::KeyPoint> model_keypoints;
	cv::Mat model_descriptors;
	std::string fname;
	
	std::unique_ptr<ObjectModel> model(new ObjectModel);
	
	std::string fs_path = expand_user("~/rapp_platform_files/") + user + "/models/";
	
	if (!model->load(fs_path, name)) return false;
	
	models.push_back(std::move(model));
	
#ifndef STRIP_ALL_ROSOUT
	if (!silent_) ROS_INFO("Loaded model: %s", name.c_str());
#endif

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

void FindObjects::storeObjectHypothesis(const ObjectHypothesis & hyp, unsigned int limit_) {

	// Special case: do not insert anything is smaller than one;)
	if (limit_<1)
		return;

	// Special case: insert first object hypothesis.
	if (hypotheses.size() == 0) {
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO("Adding first (0) object hypothesis");
#endif
		hypotheses.push_back(hyp);
		return;
	}

	// Insert in proper order.
	auto it = hypotheses.begin();
	for (; it != hypotheses.end(); ++it) {
		// check, if new object center is close to already found one
		if (norm(it->center - hyp.center) < 20) {
#ifndef STRIP_ALL_ROSOUT
			if (!silent_) ROS_INFO("Ignoring object hypothesis, to close to existing one");
#endif
			return;
		}
		if (it->score < hyp.score){
			// Insert here! (i.e. before) - stop the iterator at this place.
			break;
		}
	}
	
	hypotheses.insert(it, hyp);
#ifndef STRIP_ALL_ROSOUT
	if (!silent_) ROS_INFO("Adding next object hypothesis");
#endif
	// Limit the size of vectors.
	if (hypotheses.size() > limit_){
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO("Removing last object hypothesis");
#endif
		hypotheses.pop_back();
	}
}

bool FindObjects::verifyHypothesis(const ObjectHypothesis & hyp) {
	std::vector<double> angles(4);
	cv::Point2f tmp;

	// Compute angles.
	for (int i=0; i<4; i++) {
		tmp = (hyp.corners[i] - hyp.center);
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
	
	if ((angles[0] >= angles[1]) || (angles[1] >= angles[2]) || (angles[2] >= angles[3])) return false;
	
	if (cv::contourArea(hyp.corners) < 5000) return false;

	return true;
}

void FindObjects::refineHypothesis(ObjectHypothesis & hyp, cv::Mat scene_img) {
	cv::Mat mask = cv::Mat::zeros(scene_img.size(), CV_8UC1);
	cv::Point verts[4];
	verts[0] = cv::Point(hyp.corners[0].x, hyp.corners[0].y);
	verts[1] = cv::Point(hyp.corners[1].x, hyp.corners[1].y);
	verts[2] = cv::Point(hyp.corners[2].x, hyp.corners[2].y);
	verts[3] = cv::Point(hyp.corners[3].x, hyp.corners[3].y);
	cv::fillConvexPoly(mask, verts, 4, cv::Scalar(255, 255, 255));

	// Tmp variables.
	std::vector<KeyPoint> scene_keypoints;
	cv::Mat scene_descriptors;
	std::vector< DMatch > matches;
	
	// Extract features from scene.
	extractFeatures(scene_img, scene_keypoints, scene_descriptors, false, mask);
	
	// Find matches.
	matcher->match( hyp.model->descriptors, scene_descriptors, matches );
	
	// Filtering.
	double max_dist = 0;
	double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < matches.size(); i++ ) {
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}//: for

#ifndef STRIP_ALL_ROSOUT
	if (!silent_) ROS_INFO ("Max dist : %f", max_dist);
	if (!silent_) ROS_INFO ("Min dist : %f", min_dist);
#endif

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;

	for( int i = 0; i < matches.size(); i++ ) {
		if( matches[i].distance <= 5*min_dist+0.1 )
			good_matches.push_back( matches[i]);
	}//: for

	// Localize the object
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;
	std::vector<int> match_idx;
	std::vector<uchar> h_mask;
	int ransac_thresh = 5;

	// Get the keypoints from the good matches.
	for( int i = 0; i < good_matches.size(); i++ ) {
		obj.push_back( hyp.model->keypoints [ good_matches[i].queryIdx ].pt );
		scene.push_back( scene_keypoints [ good_matches[i].trainIdx ].pt );
		match_idx.push_back(i);
	}//: for 
	
	Mat H = findHomography( obj, scene, CV_RANSAC, ransac_thresh , h_mask);
	
	// Extract inliers
	int inliers = 0;
	for (size_t i = h_mask.size(); i > 0; --i) {
		if (h_mask[i-1] == 1) {
			++inliers;
			//used_matches.push_back(good_matches[match_idx[i-1]]);
		}
	}

	// Get the corners from the detected "object hypothesis".
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = cv::Point2f(0,0);
	obj_corners[1] = cv::Point2f( hyp.model->image.cols, 0 );
	obj_corners[2] = cv::Point2f( hyp.model->image.cols, hyp.model->image.rows );
	obj_corners[3] = cv::Point2f( 0, hyp.model->image.rows );

	// Transform corners with found homography.
	perspectiveTransform( obj_corners, hyp.corners, H);
	
	// Verification: check resulting shape of object hypothesis.
	// Compute "center of mass".
	hyp.center = (hyp.corners[0] + hyp.corners[1] + hyp.corners[2] + hyp.corners[3])*.25;

	hyp.score = (double)inliers / good_matches.size();// /models_keypoints [m].size();
}

int FindObjects::findObjects(const std::string & user, const std::string & fname, unsigned int limit, 
                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores){

	if (models.empty()) {
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_WARN("No models loaded");
#endif
		return -1;
	} else {
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO("Finding objects with the use of %d models", (int)models.size());
#endif
	}
  
#ifdef DEBUG_IMAGES
	std::string debug_path = fs::temp_directory_path().string() + "/" + fs::unique_path(user + "-%%%%-%%%%-%%%%-%%%%").string() + "/";
	fs::create_directories(debug_path);
#endif

	// Tmp variables.
	std::vector<KeyPoint> scene_keypoints;
	cv::Mat scene_descriptors;
	std::vector< DMatch > matches;

	hypotheses.clear();

	// Load image containing the scene.
	cv::Mat scene_img;
	if (!loadImage(fname, scene_img))
		return -2;

	// Extract features from scene.
	extractFeatures(scene_img, scene_keypoints, scene_descriptors, true);
#ifndef STRIP_ALL_ROSOUT
	if (!silent_) ROS_INFO ("Scene features: %d", (int)scene_keypoints.size());
#endif

	// Iterate - try to detect each model one by one.
	for (unsigned int m=0; m < models.size(); m++) {
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_DEBUG ("Trying to recognize model (%d): %s", m, models[m]->name.c_str());
#endif

		if (models[m]->keypoints.size() == 0) {
#ifndef STRIP_ALL_ROSOUT
			if (!silent_) ROS_WARN ("Model %s not valid as it does not contain texture.", models[m]->name.c_str());
#endif
			continue;
		}//: if

#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO ("Model features: %d", (int)models[m]->keypoints.size());
#endif

		std::vector< std::vector<DMatch> > knnmatches;
		// Find matches.
		matcher->match( models[m]->descriptors, scene_descriptors, matches );

#ifndef STRIP_ALL_ROSOUT		
		if (!silent_) ROS_INFO ("Matches found: %d", (int)matches.size());
#endif

		// Filtering.
		double max_dist = 0;
		double min_dist = 100;

		//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < matches.size(); i++ ) {
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}//: for

#ifdef DEBUG_IMAGES
		{
			std::ofstream of(debug_path + "matches_" + models[m]->name + ".txt");
			for( int i = 0; i < matches.size(); i++ ) {
				of << matches[i].distance << "\n";
			}
		}
#endif

#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO ("Max dist : %f", max_dist);
		if (!silent_) ROS_INFO ("Min dist : %f", min_dist);
#endif

		//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
		std::vector< DMatch > good_matches;

		for( int i = 0; i < matches.size(); i++ ) {
			if( matches[i].distance <= 5*min_dist+0.1 )
				good_matches.push_back( matches[i]);
		}//: for

#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO ("Good matches: %d", (int)good_matches.size());
#endif

#ifdef DEBUG_IMAGES
		Mat img_matches;
		drawMatches(models[m]->image, models[m]->keypoints, scene_img, scene_keypoints, good_matches, img_matches);
		imwrite(debug_path + "matches_" + models[m]->name + ".png", img_matches);
#endif

		// Localize the object
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;
		std::vector<int> match_idx;
		std::vector<uchar> h_mask;
		int ransac_thresh = 10;

		// Get the keypoints from the good matches.
		for( int i = 0; i < good_matches.size(); i++ ) {
			obj.push_back( models[m]->keypoints [ good_matches[i].queryIdx ].pt );
			scene.push_back( scene_keypoints [ good_matches[i].trainIdx ].pt );
			match_idx.push_back(i);
		}//: for 

		bool still_valid = true;
		int rep = 0;
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO ("Model (%d: %s): keypoints= %d corrs= %d", m, models[m]->name.c_str(), (int)models[m]->keypoints.size(), (int)good_matches.size());
#endif
		do { // while (still_valid)
			
			if (obj.size() < 5) break;
			
			ObjectHypothesis hyp;
			hyp.model = models[m].get();
			
			std::vector<DMatch> used_matches;
			
			// Find homography between corresponding points.
			Mat H = findHomography( obj, scene, CV_RANSAC, ransac_thresh, h_mask );

			// Extract inliers
			int inliers = 0;
			for (size_t i = h_mask.size(); i > 0; --i) {
				if (h_mask[i-1] == 1) {
					++inliers;
					used_matches.push_back(good_matches[match_idx[i-1]]);
				}
			}

			// Get the corners from the detected "object hypothesis".
			std::vector<Point2f> obj_corners(4);
			obj_corners[0] = cv::Point2f(0,0);
			obj_corners[1] = cv::Point2f( models[m]->image.cols, 0 );
			obj_corners[2] = cv::Point2f( models[m]->image.cols, models[m]->image.rows );
			obj_corners[3] = cv::Point2f( 0, models[m]->image.rows );
			hyp.corners.resize(4);

			// Transform corners with found homography.
			perspectiveTransform( obj_corners, hyp.corners, H);
			
			// Verification: check resulting shape of object hypothesis.
			// Compute "center of mass".
			hyp.center = (hyp.corners[0] + hyp.corners[1] + hyp.corners[2] + hyp.corners[3])*.25;

			hyp.score = (double)inliers / good_matches.size();// /models_keypoints [m].size();
			
			hyp.valid = verifyHypothesis(hyp);
			// Check dependency between corners.
			if (hyp.valid) {
				// Order is ok.
				if (!silent_) ROS_INFO ("   - %3d | VAL | inliers %d, score %lf", rep, inliers, hyp.score);
				// Store the model in a list in proper order.
				storeObjectHypothesis(hyp, limit);
				still_valid = true;
				// remove used matches only if match was successfull
				for (size_t i = h_mask.size(); i > 0; --i) {
					if (h_mask[i-1] == 1) {
						obj.erase(obj.begin() + (i-1));
						match_idx.erase(match_idx.begin() + (i-1));
						scene.erase(scene.begin() + (i-1));
					}
				}
			} else {
				// Hypothesis not valid.
#ifndef STRIP_ALL_ROSOUT
				if (!silent_) ROS_INFO ("   - %3d | REJ | inliers %d, score %lf", rep, inliers, hyp.score);
#endif
				still_valid = false;
				ransac_thresh *= 1.3;
			}//: else 
			
#ifdef DEBUG_IMAGES
			Mat img_object = scene_img.clone();
			line( img_object, hyp.corners[0], hyp.corners[1], Scalar(0, 255, 0), 4 );
			line( img_object, hyp.corners[1], hyp.corners[2], Scalar(0, 255, 0), 4 );
			line( img_object, hyp.corners[2], hyp.corners[3], Scalar(0, 255, 0), 4 );
			line( img_object, hyp.corners[3], hyp.corners[0], Scalar(0, 255, 0), 4 );
			circle( img_object, hyp.center, 2, Scalar(0, 255, 0), 4);
			circle( img_object, hyp.corners[0], 2, Scalar(255, 0, 0), 4);
			Mat img_matches;
			drawMatches(models[m]->image, models[m]->keypoints, img_object, scene_keypoints, used_matches, img_matches);
			float dmin = 1000;
			float dmax = 0;
			float dsum = 0;
			for (auto const & m : used_matches) {
				if (m.distance < dmin) dmin = m.distance;
				if (m.distance > dmax) dmax = m.distance;
				dsum += m.distance;
			}
			std::string s = std::to_string(dmin) + " | " + std::to_string(dsum / used_matches.size()) + " | " + std::to_string(dmax);
			cv::putText(img_matches, s, cv::Point(0, img_matches.size().height), FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar::all(255), 2, 8, false);
			imwrite(debug_path + "matches_" + models[m]->name + "_" + std::to_string(rep) + ".png", img_matches);
#endif
			if (hyp.valid) {
				refineHypothesis(hyp, scene_img);
			}
			
#ifdef DEBUG_IMAGES
			img_object = scene_img.clone();
			line( img_object, hyp.corners[0], hyp.corners[1], Scalar(0, 255, 0), 4 );
			line( img_object, hyp.corners[1], hyp.corners[2], Scalar(0, 255, 0), 4 );
			line( img_object, hyp.corners[2], hyp.corners[3], Scalar(0, 255, 0), 4 );
			line( img_object, hyp.corners[3], hyp.corners[0], Scalar(0, 255, 0), 4 );
			circle( img_object, hyp.center, 2, Scalar(0, 255, 0), 4);
			circle( img_object, hyp.corners[0], 2, Scalar(255, 0, 0), 4);
			drawMatches(models[m]->image, models[m]->keypoints, img_object, scene_keypoints, used_matches, img_matches);
			imwrite(debug_path + "matches_" + models[m]->name + "_" + std::to_string(rep) + "_ref.png", img_matches);
#endif
			
			++rep;
		} while (still_valid || rep < 3);
	}//: for

	// Copy recognized objects to output variables.
	found_names.clear();
	found_centers.clear();
	found_scores.clear();
	for(auto const & h: hypotheses){
		found_names.push_back(h.model->name);
		geometry_msgs::Point pt;
		pt.x = h.center.x;
		pt.y = h.center.y;
		pt.z = 0.0;
		found_centers.push_back(pt);
		found_scores.push_back(h.score);
#ifndef STRIP_ALL_ROSOUT
		if (!silent_) ROS_INFO("Found object: %s, %lf, %lf", h.model->name.c_str(), pt.x, pt.y);
#endif
	}//: for
 
	return 0;
}
