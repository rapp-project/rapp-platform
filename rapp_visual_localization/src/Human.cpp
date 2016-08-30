#include "rapp_visual_localization/Human.hpp"

//using namespace std;


//void CHuman::createModel(char* directoryIN, char* directoryOUT, char* modelFilenameXml,
//			bool isModel, int visualswitch, int logswitch, int monitorswitch)
//{

//}

//::cv::Rect CHuman::detectPerson(::cv::Mat obraz)
//{
//	::cv::Rect detected;


//	return detected;
//}

/**
* @brief Główna klasa służąca do generacji lub do wykrywania wzorca
*/
CHuman::CHuman(void)
{
	// Keypoint detector and extractor
	CHuman::nfeatures = 5000;//10000;
	CHuman::scaleFactor = 1.2f;
	CHuman::nlevels = 8;
	CHuman::edgeThreshold = 11;// 31;
	CHuman::firstLevel = 0;
	CHuman::WTA_K = 2;
	CHuman::scoreType = cv::ORB::FAST_SCORE; //cv::ORB::HARRIS_SCORE;
	CHuman::patchSize = 31;
	CHuman::fastThreshold = 20;
	CHuman::detector = cv::OrbFeatureDetector(nfeatures, scaleFactor, nlevels, edgeThreshold/*, firstLevel, WTA_K, scoreType, patchSize*/).create("ORB");//::FeatureDetector::create("ORB");
	CHuman::extractor = cv::DescriptorExtractor::create("ORB");
	//CHuman::DetectorExtractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel);// , WTA_K, scoreType, patchSize, fastThreshold);
	//OrbFeatureDetector detector(500, 1.2f, 8, 31, 0); // default values of 2.3.1

	// Kalman filter
	CHuman::initialized = false;
	CHuman::predict = true;
	CHuman::new_data = true;
}

void CHuman::createModel(std::vector<cv::Mat> & models_imgs,
	std::vector< std::vector<cv::Rect> > & models_rois,
	std::vector< std::vector<std::string> > & models_names_vec,
	std::string output_model_path, std::string model_file_name // sciezka do pliku z modelem
	){

	// create models.
	CHuman::createModelsFromImages(models_imgs, models_rois, models_names_vec, output_model_path, model_file_name);
}


void CHuman::loadModelFromFile(std::string path, std::string model_file_name){
	//std::string path = "/rapp_platform_files/";
	//std::string user = "Jan/";
	//path = path + user;
	std::vector< cv::KeyPoint > model_keypoints;
	// clear vectors
	CHuman::models_descriptors.clear();
	CHuman::models_keypoints.clear();
	CHuman::models_names.clear();
	if (model_file_name == "") model_file_name = "human_model.xml";
	// load model
	cv::FileStorage fs(path + model_file_name, cv::FileStorage::READ);
	fs["models_descriptors"] >> CHuman::models_descriptors;
	for (int i = 0; i < CHuman::models_descriptors.size(); i++){
		std::ostringstream oss_main;
		oss_main << i;
		std::string tmpName = "keypoints_" + oss_main.str();
		//reading vector of KeyPoints
		read(fs[tmpName], model_keypoints);

		CHuman::models_keypoints.push_back(model_keypoints);
	}

	//fs["keypoints_0"] >> model_keypoints;
	fs["models_names"] >> CHuman::models_names;
	CHuman::models_names_vec.clear();
	CHuman::models_names_vec.push_back(models_names);

	std::vector<cv::Rect> rois;
	fs["models_rois"] >> rois;
	CHuman::models_rois.clear();
	CHuman::models_rois.push_back(rois);
	fs["models_imgs"] >> CHuman::models_imgs;
	fs.release();                                       // explicit close

	/*std::cout << CHuman::models_descriptors.size() << ";"
	<< CHuman::models_names.size()
	<< CHuman::models_keypoints[0][0].pt
	<< std::endl;*/
}



std::vector< ::cv::Rect > CHuman::detectPerson(::cv::Mat obraz, std::string model_path, std::string model_name, int mode)

{
	//::cv::Rect detected;
	std::vector< cv::Rect > result_vec;

	if (mode != 2){
		int img_id = 0;
		int limit = 8;
		if (model_name == "") model_name = "human_model.xml";

		// Clear the vectors.
		//CHuman::valid_projection.clear();
		CHuman::recognized_centers.clear();
		CHuman::recognized_names.clear();
		CHuman::recognized_corners.clear();
		CHuman::recognized_scores.clear();

		// Load the model
		CHuman::loadModelFromFile(model_path, model_name);

		// Find the objects
		int detection_number = CHuman::findObjects(false, obraz, img_id,
			CHuman::models_names_vec, CHuman::models_rois, limit, CHuman::recognized_names,
			CHuman::recognized_centers, CHuman::recognized_corners, CHuman::recognized_scores,
			CHuman::recognized_frameId);

		if (detection_number > 0){
			for (int i = 0; i < CHuman::recognized_corners.size(); i++)
				result_vec.push_back(cv::Rect(CHuman::recognized_corners[i][0], CHuman::recognized_corners[i][2]));
		}

	}//: if
	if (mode != 1){
		/////////////////////////////////////////
		// HOG detector
		cv::HOGDescriptor hog;
		hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector()); // default pedestrian detector
		//int Threshold = 0;
		//int screen_width;
		//int screen_height;
		std::vector<cv::Rect> found, found_filtered;

		cv::Mat current_frame_copy;
		obraz.copyTo(current_frame_copy);
		//convert it to the gray
		cv::cvtColor(current_frame_copy, current_frame_copy, CV_BGR2GRAY);
		//std::cout<<"2"<<std::endl;
		//---------------detection with HOG algorithm---------
		hog.detectMultiScale(current_frame_copy,
			found,
			0.0,//0
			cv::Size(4, 4),//8,8
			cv::Size(16, 16),//32,32 //64,128
			1.05,//1.05
			1.0//2
			);
		//std::cout << "HOG detection Done" << std::endl;
		//filter redundant rectangles
		for (int i = 0; i < found.size(); i++)
		{
			cv::Rect r = found[i];
			int j;
			for (j = 0; j < found.size(); j++)
			if (j != i && (r & found[j]) == r)
				break;
			if (j == found.size())
				found_filtered.push_back(r);

			// display
			//for (int i = 0; i<found.size(); i++)
			//	cv::rectangle(current_frame_copy, found[i], cv::Scalar(0, 255, 0), 3);
		}
		//number_of_people = found_filtered.size();
		//std::cout << "Number of detected humans: " << number_of_people << std::endl;
		// draw rectangles
		//for (int i = 0; i<found_filtered.size(); i++)
		//	cv::rectangle(current_frame_copy, found_filtered[i], cv::Scalar(255, 0, 0), 3);
		
		if (mode == 2){
			result_vec = found_filtered;
		}
		else {
			//for (int i = 0; i < found_filtered.size(); i++)
			//	result_vec.push_back(found_filtered[i]);
			//filter redundant rectangles
			for (int i = 0; i < found_filtered.size(); i++)
			{
				cv::Rect r = found_filtered[i];
				int j;
				for (j = 0; j < found_filtered.size(); j++)
				if (j != i && (r & found_filtered[j]) == r)
					break;
				if (j == found_filtered.size())
					result_vec.push_back(r);
			}
		}
	}

	return result_vec;
}


int CHuman::findObjects(const bool isMultiple, cv::Mat & img_, int img_id,
	std::vector< std::vector< std::string > > & models_names_, std::vector< std::vector<cv::Rect> > models_rois_,
	unsigned int limit,
	std::vector<std::string> & found_names, std::vector<cv::Point2f> & found_centers,
	std::vector<std::vector<cv::Point2f> > &found_corners, std::vector<double> & found_scores, std::vector<int> & recognized_frameId_){

	int knn = 5;// 3; // Count of best matches found per each query descriptor or less if a query descriptor has less than k possible matches in total.

	// variable for good matches selection
	const double min_dst_ext = 5; // distance for 'good' matches evaluation
	const int min_good_match_count = 4; //atleast min_good_match_count matches are to be there to find the object
	int detected_models = 0;

	const bool showText = false; // if true, then text messages will be shown
	const bool showProjection = true; // if showTime will be equal to 'true' than projection will not be shown

	const bool showTime = false;// true;// true;// false;// true; // Time duration for object(model) finding // showProjection should be equal to 'false'
	double time0, time1;
	std::vector< double > detectionTimeList;
	std::vector< std::vector< double > > detectionTimeLists;


	///Finding objects with the use of models_names_.size() models

	// Temporary variables.
	std::vector<cv::KeyPoint> scene_keypoints;
	cv::Mat scene_descriptors;
	std::vector< cv::DMatch > matches;
	std::vector< std::vector< cv::DMatch > > matches_knn;

	/// Extract features from scene.
	CHuman::extractFeatures(img_, scene_keypoints, scene_descriptors);

	// Iterate - try to detect each model one by one.
	for (unsigned int n = 0; n < models_names_.size(); n++) {
		std::vector< std::string > detected_model; // for recognized models

		if (showTime){
			time0 = (double)cv::getTickCount();//initial time measure
			detectionTimeList.clear();//clear the vector
		}

		//for (unsigned int o = 0; o < models_names_.size(); o++) {
		if (n < models_names_.size()) {
			/// Extract features from scene.
			//extractFeatures(img_, scene_keypoints, scene_descriptors);

			if (showText){
				std::cout << "--------------------" << std::endl;
				std::cout << "Scene features: " << scene_keypoints.size() << std::endl;
				//std::cout << "Images: " << imgs_.size() << std::endl;
				std::cout << "Size of vector models_names: " << models_names_.size() << std::endl;
				std::cout << "Size of vector models_names[" << n << "]: " << models_names_[n].size() << std::endl;
				std::cout << "--------------------" << std::endl;
			}//: if

			// Release vector containing center points of detected models
			//found_centers.clear();

			for (unsigned int m = 0; m < models_names_[n].size(); m++) {
				if (showTime){
					time1 = (double)cv::getTickCount(); //initial time measure for each model name
				}
				if (showText) std::cout << "Trying to recognize model (" << m << "): " << models_names_[n][m] << std::endl;

				if ((CHuman::models_keypoints[m]).size() == 0) {
					std::cerr << "Model " << models_names_[n][m].c_str() << " not valid as it does not contain texture." << std::endl;
					continue;
				}//: if

				if (showText) std::cout << "Model features: " << CHuman::models_keypoints[m].size() << std::endl;

				//(single object instance detection)
				{
					//// MATCHING
					/// Matching descriptor vectors using FLANN or Brute-Force matcher
					// Find matches
					//matcher.match(models_descriptors[m], scene_descriptors, matches); //matching // BF is faster for binary descriptors

					// cross check matching
					CHuman::crossCheckMatching(descriptorMatcher, CHuman::models_descriptors[m], scene_descriptors, matches/*filteredMatches*/, knn);


					/// Filtering variables
					// Quick calculation of max and min distances between keypoints
					// Compute min/max value
					auto result = std::minmax_element(matches.begin(), matches.end());
					double min_dist = matches[result.first - matches.begin()].distance;

					if (showText){
						std::cout << "Matches found: " << matches.size() << std::endl;
						std::cout << "Min dist : " << min_dist << std::endl;
					}

					//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist or less than 35.0)
					std::vector< cv::DMatch > good_matches;
					for (int i = 0; i < matches.size(); i++) {
						// Select good matches
						if (matches[i].distance < std::max(std::min(3 * min_dist, 35.0), min_dst_ext))
							good_matches.push_back(matches[i]);
					}//: for
					if (showText) std::cout << "Good matches: " << good_matches.size() << std::endl;
					////

					// Localize the object
					std::vector<cv::Point2f> obj;
					std::vector<cv::Point2f> scene;
					for (int i = 0; i < good_matches.size(); i++) {
						// Get the keypoints from the good matches
						obj.push_back(CHuman::models_keypoints[m][good_matches[i].queryIdx].pt);
						scene.push_back(scene_keypoints[good_matches[i].trainIdx].pt);
					}//: for

					/// Compute homography
					if (good_matches.size() >= min_good_match_count){

						// Find homography between corresponding points
						cv::Mat mask;
						cv::Mat H = cv::findHomography(obj, scene, /*cv::RHO*/CV_RANSAC, 1.0, mask/*inliers */);// RHO - PROSAC-based robust method 

						if (H.empty()){ // Homography was not found
							std::cout << "Homography was not found (good matches < 4)\n" << std::endl;
							continue; // skip evaluation of perspective transformation
						}

						// Get the corners from the detected "object hypothesis".
						std::vector<cv::Point2f> obj_corners(4);
						obj_corners[0] = cv::Point2f(0, 0);
						obj_corners[1] = cv::Point2f(models_rois_[n][m].width, 0); // model instead of img_
						obj_corners[2] = cv::Point2f(models_rois_[n][m].width, models_rois_[n][m].height);
						obj_corners[3] = cv::Point2f(0, models_rois_[n][m].height);
						std::vector<cv::Point2f> hypobj_corners(4);

						// Transform corners with found homography.
						cv::perspectiveTransform(obj_corners, hypobj_corners, H);

						// Verification: check resulting shape of object hypothesis.
						// Compute "center of mass".
						cv::Point2f center = (hypobj_corners[0] + hypobj_corners[1] + hypobj_corners[2] + hypobj_corners[3])*.25;
						std::vector<double> angles(4);
						cv::Point2f tmp;

						// store center point of detected model
						//found_centers.push_back(center);

						// Compute angles.
						for (int i = 0; i < 4; i++) {
							tmp = (hypobj_corners[i] - center);
							angles[i] = atan2(tmp.y, tmp.x);
						}//: for

						// Find smallest element.
						int imin = -1;
						//double amin = 1000;
						double amin = nfeatures;
						for (int i = 0; i<4; i++)
						if (amin > angles[i]) {
							amin = angles[i];
							imin = i;
						}//: if

						// Reorder table.
						for (int i = 0; i < imin; i++) {
							angles.push_back(angles[0]);
							angles.erase(angles.begin());
						}//: for

						//std::cout << "Center: " << center << "\n";

						double score = (double)good_matches.size() / models_keypoints[m].size();
						// Check dependency between corners.
						// Check if Hypothesis (Projection) is valid
						if ((angles[0] < angles[1]) && (angles[1] < angles[2]) && (angles[2] < angles[3])) {
							// Order is ok.
							std::cout << "Model (" << m << "): keypoints= " << models_keypoints[m].size() << " good matches = " << good_matches.size() << " score = " << score << " VALID" << std::endl;
							// Store the model in a list in proper order.
							CHuman::storeObjectHypothesis(models_names_[n][m], center, hypobj_corners, score, img_id, limit,
								found_names, found_centers, found_corners, found_scores, recognized_frameId_);

							//valid_projection.push_back(true); // projection is valid
							detected_model.push_back(models_names_[n][m]); // model name is stored
							detected_models++;

							// Show projection.
							if ((!showTime) && showProjection){
								// draw inliers if object was succesfully found
								cv::Mat img_matches;
								//cv::drawKeypoints(imgs_[n], scene_keypoints, img_matches, cv::Scalar::all(-1), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
								cv::drawKeypoints(img_, scene_keypoints, img_matches, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
								cv::imshow("Keypoints", img_matches.clone());
								//std::cout << models_imgs.size() << "; " << models_rois.size() << "; " << n << ", " << models_rois[n].size() << ", "<< m<<std::endl;
								cv::Mat roi = models_imgs[n](models_rois_[n][m]);
								cv::drawMatches(roi, models_keypoints[m], img_, scene_keypoints, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
								cv::imshow("Matching", img_matches.clone());
								cv::waitKey(1); // show image and wait for a keystroke
							}

						}//: if
						else {
							// Hypothesis not valid.
							std::cout << "Model (" << m << "): keypoints= " << models_keypoints[m].size() << " good matches = " << good_matches.size() << " score = " << score << " REJECTED" << std::endl;
							//valid_projection.push_back(false); // projection is not valid
							std::cout << "Angles: " << angles[0] << "; " << angles[1] << "; " << angles[2] << "; " << angles[3] << std::endl; //test

							if (showProjection && !showTime){
								// draw inliers if object was succesfully found
								cv::Mat img_matches;
								//cv::drawKeypoints(imgs_[n], scene_keypoints, img_matches, cv::Scalar::all(-1), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
								cv::drawKeypoints(img_, scene_keypoints, img_matches, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
								cv::namedWindow("Keypoints", cv::WINDOW_AUTOSIZE);// Create a window for display.
								cv::imshow("Keypoints", img_matches.clone());
								//std::cout << models_imgs.size() << "; " << models_rois.size() << "; " << n << ", " << models_rois[n].size() << ", "<< m<<std::endl;
								cv::Mat roi = models_imgs[n](models_rois_[n][m]);
								cv::drawMatches(roi, models_keypoints[m], img_, scene_keypoints, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
								cv::namedWindow("Matching", cv::WINDOW_AUTOSIZE);// Create a window for display.
								cv::imshow("Matching", img_matches.clone());
								if (!showTime) cv::waitKey(1); // waits 1ms -- showing image
							}//: if
						}//: else
					}//: if
				}//: else (single object instance detection)
			}//: for models[n]
		}//: if

	}//: for models

	// Clear the vectors
	scene_keypoints.clear();
	matches.clear();
	scene_descriptors.release();

	return detected_models;//0;
}


bool CHuman::extractFeatures(const cv::Mat image_, std::vector<cv::KeyPoint> & keypoints_, cv::Mat & descriptors_){

	cv::Mat gray_img;

	cv::Mat grad;
	int featureId = 1;//1; //1 --ORB, 2 --AKAZE/KAZE
	int dilation_size = 1;// 2; // size of structuring element (2 * dilation_size + 1, 2 * dilation_size + 1)
	const bool computeGradients = false; // true;// false;
	const bool showTime = true; // Time duration of detection and extraction
	double time0, time1;

	try {
		// Transform to grayscale - if requred.
		if (image_.channels() == 1){
			gray_img = image_.clone();

			/// Gradient filtering -- for one channel
			if (computeGradients) cv::morphologyEx(image_, grad, CV_MOP_GRADIENT, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1)));
		}
		else{
			cv::cvtColor(image_, gray_img, cv::COLOR_BGR2GRAY);
		}

		/// Compute gradients
		/// Gradient filtering
		if (computeGradients)
			try {
			cv::Mat img_yuv;
			std::vector< cv::Mat > channelVector;
			cv::cvtColor(image_, img_yuv, CV_BGR2YUV); // Convert BGR to YUV color space

			cv::split(img_yuv, channelVector); // splitting to channels
			/// Normalize luminance
			// apply the CLAHE algorithm to the L channel
			cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
			clahe->setClipLimit(4);
			clahe->apply(channelVector[0], img_yuv);// normalize luminance

			cv::morphologyEx(img_yuv, grad, CV_MOP_GRADIENT, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1)));

			//cv::imshow("Gradient", grad); //not needed
			//cv::waitKey(1); //not needed
		}
		catch (...)
		{
			std::cout << "MorphologyEX\n";
		}

		/// Detect and Extract
		// Detect the keypoints.
		keypoints_.clear(); // clearing the keypoint vector
		if (showTime) time0 = (double)cv::getTickCount();//initial time measure

		// KAZE/AKAZE features - KAZE/AKAZE features are state-of-art in terms of robustness and localization accuracy
		//	cv::Ptr<cv::AKAZE> f2d = cv::AKAZE::create();
		//cv::Ptr<cv::KAZE> f2d = cv::KAZE::create();
		// Feature detection
		if (!computeGradients){
			/*if (featureId == 2){
			// AKAZE features
			f2d->detect(gray_img, keypoints_, cv::noArray());
			}
			else*/{
			// ORB
			//CHuman::DetectorExtractor->detect(gray_img, keypoints_, cv::noArray());
			CHuman::detector->detect(gray_img, keypoints_, descriptors_);
		}
		}
		else {
			/*if (featureId == 2){
			// AKAZE features
			f2d->detect(grad, keypoints_, cv::noArray());
			}
			else*/{
			// ORB
			//CHuman::DetectorExtractor->detect(grad, keypoints_, cv::noArray());
			CHuman::detector->detect(gray_img, keypoints_, descriptors_);
		}
		}

		if (showTime){
			time1 = ((double)cv::getTickCount() - time0) / cv::getTickFrequency();
			std::cout << "DETECTOR - Times passed in seconds: " << time1 << std::endl;
		}

		// Extract descriptors (feature vectors).
		if (!computeGradients){
			/*if (featureId == 2){
			// AKAZE features
			f2d->compute(gray_img, keypoints_, descriptors_);
			}
			else*/{
			// ORB
			//CHuman::DetectorExtractor->compute(gray_img, keypoints_, descriptors_);
			CHuman::extractor->compute(gray_img, keypoints_, descriptors_);
		}
		}
		else {
			/*if (featureId == 2){
			// AKAZE features
			f2d->compute(grad, keypoints_, descriptors_);
			}
			else*/{// ORB
			//CHuman::DetectorExtractor->compute(grad, keypoints_, descriptors_);
			CHuman::extractor->compute(gray_img, keypoints_, descriptors_);
		}
		}
		if (showTime){
			time1 = ((double)cv::getTickCount() - time0) / cv::getTickFrequency();
			//std::cout << "DETECTOR and EXTRACTOR - Times passed in seconds: " << time1 << std::endl;
		}

		return true;
	}//: try
	catch (...) {
		std::cerr << "Could not extract features from image" << std::endl;
		return false;
	}//: catch
}

void CHuman::createModelsFromImages(std::vector<cv::Mat> & images_, std::vector< std::vector<cv::Rect> > & rois_,
	std::vector< std::vector<std::string> > & names_,
	std::string output_model_path, std::string model_file_name // sciezka do pliku z modelem
	){
	// on the given ROIs, extracting features and adding them to the model

	cv::Mat model_img;
	const bool show_info = false;//true;

	// Clear database.
	//models_imgs.clear();
	CHuman::models_keypoints.clear();
	CHuman::models_descriptors.clear();
	CHuman::models_names.clear();
	CHuman::models_names_vec = names_;
	CHuman::models_rois = rois_;
	CHuman::models_imgs = images_;

	//for (unsigned int n = 0; n < names_.size(); n++)
	//for (unsigned int m = 0; m < names_[n].size(); m++)
	//	std::cout << names_[n][m] << std::endl;

	// Create all models - extract feature for all given ROIs and add data to lists.
	for (int i = 0; i<images_.size(); i++) {
		// clear temporary vector
		CHuman::models_names.clear();

		//model_img = images_[i]; // for the operation on the image
		for (int j = 0; j < rois_[i].size(); j++)
		{
			if (names_[i].size() != rois_[i].size()){ // size of this two vectors should be equal
				std::cerr << "Size of names_[" << i << "] !=size of rois_[" << i << "], but it should be equal" << std::endl;
				break;
			}//: if
			else {
				\
					/*
					// Add 16px, to the selection, in all general directions
					int x_tmp = rois_[i][j].x -16;
					int width_tmp = rois_[i][j].width + 32;
					int y_tmp = rois_[i][j].y;// -16;
					int height_tmp = rois_[i][j].height;// +32;
					if (height_tmp + y_tmp > images_[i].rows) height_tmp = images_[i].rows - y_tmp;
					if (width_tmp + x_tmp > images_[i].cols) width_tmp = images_[i].cols - x_tmp;
					if (x_tmp < 0) x_tmp = 0;
					if (y_tmp < 0) y_tmp = 0;


					cv::Rect roi_tmp = cv::Rect(x_tmp, y_tmp, width_tmp, height_tmp);
					//std::cout <<"ROI: " << rois_[i][j] << std::endl; //displays ROIs
					cv::Mat roi = images_[i](roi_tmp);//model_img(rois_[i][j]);
					*/
					cv::Mat roi = images_[i](rois_[i][j]);
				std::vector<cv::KeyPoint> model_keypoints;
				cv::Mat model_descriptors;
				CHuman::extractFeatures(roi, model_keypoints, model_descriptors); //extracting the Features for the ROI

				// Add to "database". -- for i-th object
				//models_imgs.push_back(model_img);
				CHuman::models_keypoints.push_back(model_keypoints);
				CHuman::models_descriptors.push_back(model_descriptors);
				CHuman::models_names.push_back(names_[i][j]);
				//std::cout << "test" << i << "," << j << std::endl;//

				if (show_info)
				{
					std::cout << "--------------------" << std::endl;
					std::cout << names_[i][j] << ":\n\tkeypoins: \t" << model_keypoints.size() << std::endl;
					std::cout << "\tdescriptors: \t" << model_descriptors.size() << std::endl;
					std::cout << "--------------------" << std::endl;
				}
			}//: else

		}//: for
		if (!models_names.empty()){
			//CHuman::models_names_vec[i] = models_names; // adding models names
		}//: if
	}//: for

	// Save the generated model to the file
	cv::FileStorage fs(output_model_path+model_file_name, cv::FileStorage::WRITE);
	for (int i = 0; i < models_descriptors.size(); i++){
		//fs << "keypoints_" + i << CHuman::models_keypoints[i];

		std::ostringstream oss_main;
		oss_main << i;
		std::string tmpName = "keypoints_" + oss_main.str();
		//writing 
		write(fs, tmpName, CHuman::models_keypoints[i]);
	}
	std::vector<cv::Rect> rois;
	for (int i = 0; i < rois_.size(); i++)
	for (int j = 0; j < rois_[i].size(); j++)
		rois.push_back(rois_[i][j]);
	fs << "models_rois" << rois;
	fs << "models_imgs" << images_;

	fs << "models_descriptors" << CHuman::models_descriptors;
	fs << "models_names" << CHuman::models_names;
	fs.release();                                       // explicit close

	CHuman::models_keypoints.clear();
	CHuman::models_descriptors.clear();
	CHuman::models_names.clear();
	CHuman::models_names_vec.clear();
	//CHuman::models_rois.clear(); //<--
	//CHuman::models_imgs.clear(); //<--
	//system("pause");
	//CHuman::loadModelFromFile();
	//system("pause");
}

void CHuman::storeObjectHypothesis(std::string name_, cv::Point2f center_, std::vector<cv::Point2f> corners_, double score_, int frameId_, unsigned int limit_,
	std::vector<std::string> &recognized_names_, std::vector<cv::Point2f> &recognized_centers_,
	std::vector<std::vector<cv::Point2f> > &recognized_corners_, std::vector<double> &recognized_scores_,
	std::vector<int> & recognized_frameId_){

	// Special case: do not insert anything is smaller than one;)
	//if (limit_ < 1)
	//	return;

	// Special case: insert first object hypothesis.
	if (recognized_names_.size() == 0) {
		recognized_names_.push_back(name_);
		recognized_centers_.push_back(center_);
		recognized_corners_.push_back(corners_);
		recognized_scores_.push_back(score_);
		recognized_frameId_.push_back(frameId_);
		return;
	}//: if

	// Iterators.
	std::vector<std::string>::iterator names_it = recognized_names_.begin();
	std::vector<cv::Point2f>::iterator centers_it = recognized_centers_.begin();
	std::vector<std::vector<cv::Point2f> >::iterator corners_it = recognized_corners_.begin();
	std::vector<double>::iterator scores_it = recognized_scores_.begin();
	std::vector<int>::iterator frame_it = recognized_frameId_.begin();

	bool added = false;
	// Insert in proper order.
	for (; names_it != recognized_names_.end(); names_it++, centers_it++, corners_it++, scores_it++, frame_it++) {
		if (*scores_it < score_){
			// Insert here! (i.e. before) - stop the iterator at this place.
			break;
		}//: if
	}//: for
	std::cout << "Adding next object hypothesis" << std::endl;
	recognized_names_.insert(names_it, name_);
	recognized_centers_.insert(centers_it, center_);
	recognized_corners_.insert(corners_it, corners_);
	recognized_scores_.insert(scores_it, score_);
	recognized_frameId_.insert(frame_it, frameId_);

	// Limit the size of vectors.
	if (recognized_names_.size() > limit_ && limit_>0){
		std::cout << "Removing last object hypothesis" << std::endl;
		recognized_names_.pop_back();
		recognized_centers_.pop_back();
		recognized_corners_.pop_back();
		recognized_scores_.pop_back();
		recognized_frameId_.pop_back();
	}//: if
}

// matching with a cross check
void CHuman::crossCheckMatching(cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
	const cv::Mat& descriptors1, const cv::Mat& descriptors2,
	std::vector<cv::DMatch>& filteredMatches12, int knn){

	filteredMatches12.clear();
	std::vector<std::vector<cv::DMatch> > matches12, matches21;

	if (descriptors1.type() == CV_8U){
		// binary descriptors detected
		cv::BFMatcher matcher_for_knn(cv::NORM_HAMMING, false); // with cross-checks disabled
		matcher_for_knn.knnMatch(descriptors1, descriptors2, matches12, knn);
		matcher_for_knn.knnMatch(descriptors2, descriptors1, matches21, knn);
	}
	else {
		// assume it is CV_32F
		// float descriptors detected
		cv::BFMatcher matcher_for_knn(cv::NORM_L2, false); // with cross-checks disabled
		matcher_for_knn.knnMatch(descriptors1, descriptors2, matches12, knn);
		matcher_for_knn.knnMatch(descriptors2, descriptors1, matches21, knn);
	}

	// cross-check
	for (size_t m = 0; m < matches12.size(); m++)
	{
		bool findCrossCheck = false;
		for (size_t fk = 0; fk < matches12[m].size(); fk++)
		{
			cv::DMatch forward = matches12[m][fk];
			for (size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++)
			{
				cv::DMatch backward = matches21[forward.trainIdx][bk];
				if (backward.trainIdx == forward.queryIdx)
				{
					filteredMatches12.push_back(forward);
					findCrossCheck = true;
					break;
				}
			}
			if (findCrossCheck) break;
		}
	}
}

// Process Nearest Neighbor Distance Ratio
int CHuman::processNNDR(std::vector<std::vector<cv::DMatch> >& matches, std::vector<cv::KeyPoint>& objectKeypoints, std::vector<cv::KeyPoint>& sceneKeypoints,
	cv::Mat & img_, int img_id, // information about image frame
	std::string & model_name_, // model name
	unsigned int minInliers, // minimal number of inliers (>=4)
	cv::Rect models_rois,
	unsigned int limit, // Limit the size of the received vectors
	std::vector<std::string> & found_names, std::vector<cv::Point2f> & found_centers, std::vector<std::vector<cv::Point2f> > &recognized_corners_, std::vector<double> & found_scores, std::vector<int> & recognized_frameId_){// vectors with a detection results

	std::vector< cv::Mat > homographyVector; // for multiple instances of the object

	bool showInliers = true;// if true and showDetection is also true, then inliers and outliers will be shown
	bool showDetection = true;// false; // if true, then Img with all valid detections will be shown

	// detection number of the object hipothesis
	int detected_model = 0;

	if (minInliers < 4) minInliers = 4; // minimum 4 points are needed for the homography computation

	float nndrRatio = 0.8f;
	std::vector< cv::Point2f > mptsObj, mptsScene;	// for homography
	std::vector< int > indexesObj, indexesScene;	// for homography
	std::vector< uchar > outlierMask;				// mask for homography
	cv::Mat H;										// Homography matrix

	// Check if the descriptor matches with those of the object model
	for (unsigned int i = 0; i < matches.size(); i++){
		// Apply NNDR (Nearest Neighbor Distance Ratio)
		if (matches.at(i).size() /*==*/ >= 2 && // n
			matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance) // check the distance ratio
		{
			mptsObj.push_back(objectKeypoints.at(matches.at(i).at(0).queryIdx).pt);
			indexesObj.push_back(matches.at(i).at(0).queryIdx);

			mptsScene.push_back(sceneKeypoints.at(matches.at(i).at(0).trainIdx).pt);
			indexesScene.push_back(matches.at(i).at(0).queryIdx);
		}
	}

	// Find Homography
	//unsigned int minInliers = 8;
	if (mptsObj.size() >= minInliers)
	{
		// compute homography -- the perspective transformation H between the source and the destination planes
		H = cv::findHomography(mptsObj, //Coordinates of the points in the original plane
			mptsScene,					//Coordinates of the points in the target plane
			cv::RANSAC,//cv::RHO,//
			1.0,
			outlierMask);

		// kopiuj obraz
		cv::Mat img;//
		if (showDetection) img = img_.clone();
		//cv::imshow("IMG", img);
		//cv::waitKey(1);

		// detect multiple instances of the object
		try {
			while (!H.empty()){
				std::vector< cv::Point2f > mptsObjNext, mptsSceneNext;// for the next instance of the object
				int inliers = 0, outliers = 0;
				for (unsigned int i = 0; i < mptsObj.size(); i++){
					if (outlierMask.at(i)){ // 
						++inliers; // Inliers
						//std::cout << i << " green; " <<
						//	mptsObj[i].x << ", " << mptsObj[i].y << "; ";
						if (showDetection && showInliers){
							// Wyświetl hipotezę obiektu
							cv::rectangle(img, cv::Rect(mptsScene[i], cv::Point(mptsScene[i].x + 1, mptsScene[i].y + 1)), cv::Scalar(0, 255, 0), 5);
						}
					}
					else {
						// copy the points which are outliers -- for the usage in the next iteration
						mptsObjNext.push_back(mptsObj[i]);
						mptsSceneNext.push_back(mptsScene[i]);

						++outliers; // Outliers (Obserwacje odstające)

						if (showDetection && showInliers){
							// Wyświetl hipotezę sceny
							cv::rectangle(img, cv::Rect(mptsScene[i], cv::Point(mptsScene[i].x + 1, mptsScene[i].y + 1)), cv::Scalar(0, 0, 255), 5);
						}
					}
				}

				// Breaks if number of inliers is smaller than minInliers
				if (inliers < minInliers){
					//std::cout << inliers << "<" << minInliers << " minInliers\n";
					break;
				}
				//

				// Get the corners from the detected "object hypothesis".
				std::vector<cv::Point2f> obj_corners(4);
				obj_corners[0] = cv::Point2f(0, 0);
				obj_corners[1] = cv::Point2f(models_rois.width, 0); // model instead of img_
				obj_corners[2] = cv::Point2f(models_rois.width, models_rois.height);
				obj_corners[3] = cv::Point2f(0, models_rois.height);
				std::vector<cv::Point2f> hypobj_corners(4);

				if (!H.empty()) // not empty
				{
					// Transform corners with found homography.
					cv::perspectiveTransform(obj_corners, hypobj_corners, H);

					// Wyświetl hipotezę obiektu na obrazie
					//cv::Mat img = img_.clone();
					//resize(img, img, cv::Size(), 5.0 / 10.0, 5.0 / 10.0);//resize the image
					//std::cout << hypobj_corners[0] << "; " << hypobj_corners[2] << "\n"; // for test

					// Verification: check resulting shape of object hypothesis.
					// Compute "center of mass".
					cv::Point2f center = (hypobj_corners[0] + hypobj_corners[1] + hypobj_corners[2] + hypobj_corners[3])*.25;
					std::vector<double> angles(4);
					cv::Point2f tmp;

					// Compute angles.
					for (int i = 0; i < 4; i++) {
						tmp = (hypobj_corners[i] - center);
						angles[i] = atan2(tmp.y, tmp.x);
					}//: for

					// Find smallest element.
					int imin = -1;
					//double amin = 1000;
					double amin = nfeatures;
					for (int i = 0; i<4; i++)
					if (amin > angles[i]) {
						amin = angles[i];
						imin = i;
					}//: if

					// Reorder table.
					for (int i = 0; i < imin; i++) {
						angles.push_back(angles[0]);
						angles.erase(angles.begin());
					}//: for

					//std::cout << "Center: " << center << "\n";

					double score = inliers / mptsObj.size();
					// Check dependency between corners.
					// Check if Hypothesis (Projection) is valid
					if ((angles[0] < angles[1]) && (angles[1] < angles[2]) && (angles[2] < angles[3])) {
						// Order is ok.
						//std::cout << "Model (" << m << "): keypoints= " << models_keypoints[m].size() << " good matches = " << good_matches.size() << " score = " << score << " VALID" << std::endl;
						if (showDetection){
							cv::rectangle(img, cv::Rect(hypobj_corners[0], hypobj_corners[2]), cv::Scalar(125, 125, 0), 3);
							cv::imshow("Detected", img);
							cv::waitKey(1);
						}
						//std::cout << " VALID" << std::endl;

						CHuman::storeObjectHypothesis(model_name_, center, hypobj_corners, score, img_id, limit,
							found_names, found_centers, recognized_corners_, found_scores, recognized_frameId_);

						//	CHuman::valid_projection.push_back(true); // projection is valid
						detected_model++;
					}//: if
				}//: if

				// Breaks if new homography could not be computed
				if (mptsObjNext.size() < 4)
					break;

				// compute homography -- the perspective transformation H between the source and the destination planes
				H = cv::findHomography(mptsObjNext, //Coordinates of the points in the original plane
					mptsSceneNext,					//Coordinates of the points in the target plane
					cv::RANSAC,//cv::RHO,//
					1.0,
					outlierMask);
				mptsObj = mptsObjNext;
				mptsScene = mptsSceneNext;
				//system("pause"); // wait for a keystroke
				//cv::waitKey(0);
			}//: while
		}//: try
		catch (...)
		{
			std::cerr << "Błąd w wyznaczaniu homografii\n";
		}//: catch
		// Hypothesis detection of the object was ended
		std::cout << "Detekcja hipotez obiektu została zakończona\n";
		//system("pause"); // wait for a keystroke
	}
	else{
		std::cout << "Za mało znalezionych par punktów do wyznaczenia homografii\n";
	}
	return detected_model; // returns number of found object hypothesis
}


cv::Mat CHuman::computeHumanToCamera(double camera_matrix[][3],
	double TheoreticalHeight,
	double TheoreticalWidth,
	//std::vector< cv::Point2f > humanRect,
	cv::Rect humanRect,
	cv::Mat robotToCameraMat){

	std::vector<cv::Point3f> object_points;
	std::vector<cv::Point2f> pixel_coords;
	//int TheoreticalHeight;
	//int TheoreticalWidth;
	// Model for SolvePnP // x-> y^
	object_points.clear();

	//TheoreticalWidth = 0.16*2; // test
	//TheoreticalHeight = 0.16*2; // test

	// 
	if (humanRect.width>0){//humanRect.size() == 4){
		/*pixel_coords.push_back(humanRect[0]); //top left
		pixel_coords.push_back(humanRect[3]);
		pixel_coords.push_back(humanRect[2]);
		pixel_coords.push_back(humanRect[1]); // top right
		*/
		pixel_coords.push_back(cv::Point(humanRect.x, humanRect.y)); //top left
		pixel_coords.push_back(cv::Point(humanRect.x, humanRect.y + humanRect.height));
		pixel_coords.push_back(cv::Point(humanRect.x + humanRect.width, humanRect.y + humanRect.height));
		pixel_coords.push_back(cv::Point(humanRect.x + humanRect.width, humanRect.y)); // top right
	}
	//pixel_coords.push_back(cv::Point2f(humanRect.x, humanRect.y)); //top left
	//pixel_coords.push_back(cv::Point2f(humanRect.x, humanRect.y + humanRect.height));
	//pixel_coords.push_back(cv::Point2f(humanRect.x + humanRect.width, humanRect.y + humanRect.height));
	//pixel_coords.push_back(cv::Point2f(humanRect.x + humanRect.width, humanRect.y)); // top right

	//z^y<-
	object_points.push_back(cv::Point3f(0, TheoreticalWidth / 2, TheoreticalHeight));
	object_points.push_back(cv::Point3f(0, TheoreticalWidth / 2, 0));
	object_points.push_back(cv::Point3f(0, -TheoreticalWidth / 2, 0));
	object_points.push_back(cv::Point3f(0, -TheoreticalWidth / 2, TheoreticalHeight));

	//std::cout << humanRect[0].y << ";" << humanRect[1].y << ";;" << humanRect[3].y << std::endl; //test

	// Camera Intrinsic Matrix -- from Camera calibration
	cv::Mat cameraIntrinsicMatrix;
	cv::Mat rvec;//(3,1,cv::DataType<float>::type);
	cv::Mat tvec;//(3,1,cv::DataType<float>::type);
	cv::Mat rotationMatrix;//(3,3,cv::DataType<float>::type);
	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
	cv::Mat humanToCameraTransform = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
	cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_matrix);

	cv::solvePnP(cv::Mat(object_points), cv::Mat(pixel_coords), cameraIntrinsicMatrix, distCoeffs, rvec, tvec, false);//, CV_ITERATIVE );

	cv::Rodrigues(rvec, rotationMatrix);


	// humanToCameraTransform computation
	for (int i = 0; i<3; i++)
	{
		for (int j = 0; j<3; j++)
			humanToCameraTransform.at<double>(i, j) = rotationMatrix.at<double>(i, j);
	}
	humanToCameraTransform.at<double>(0, 3) = tvec.at<double>(0, 0); //x->y^ : x
	humanToCameraTransform.at<double>(1, 3) = tvec.at<double>(1, 0); //x->y^ : y
	humanToCameraTransform.at<double>(2, 3) = tvec.at<double>(2, 0); //x->y^ : z
	humanToCameraTransform.at<double>(3, 3) = 1.0;

	cv::Mat Rotx_minus90 = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
	cv::Mat Rotz_minus90 = cv::Mat::zeros(4, 4, cv::DataType<double>::type);

	Rotx_minus90.at<double>(0, 0) = 1.f;
	Rotx_minus90.at<double>(1, 1) = 0.f; Rotx_minus90.at<double>(1, 2) = +1.f;
	Rotx_minus90.at<double>(2, 1) = -1.f; Rotx_minus90.at<double>(2, 2) = 0.f;
	Rotx_minus90.at<double>(3, 3) = 1.f;

	Rotz_minus90.at<double>(0, 0) = 0.f; Rotz_minus90.at<double>(0, 1) = +1.f;
	Rotz_minus90.at<double>(1, 0) = -1.f; Rotz_minus90.at<double>(1, 1) = 0.f;
	Rotz_minus90.at<double>(2, 2) = 1.f;
	Rotz_minus90.at<double>(3, 3) = 1.f;


	//#####################			
	//## Transformation from the QR-code coordinate system to the camera coordinate system
	cv::Mat cameraToLandmarkTransformMatrix = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
	cameraToLandmarkTransformMatrix = Rotz_minus90*Rotx_minus90*humanToCameraTransform;
	//#####################

	CHuman::new_data = true; // new position was given (for the prediction)

	//#####################						
	//## Transformation from the camera coordinate system to the robot coordinate system
	cv::Mat robotToLandmarkMatrix;
	if (!robotToCameraMat.empty()){
		robotToLandmarkMatrix = robotToCameraMat*cameraToLandmarkTransformMatrix;
		CHuman::human_position_x = robotToLandmarkMatrix.at<double>(0, 3);
		CHuman::human_position_y = robotToLandmarkMatrix.at<double>(1, 3);
		return robotToLandmarkMatrix;
	}
	CHuman::human_position_x = cameraToLandmarkTransformMatrix.at<double>(0, 3);
	CHuman::human_position_y = cameraToLandmarkTransformMatrix.at<double>(1, 3);
	return cameraToLandmarkTransformMatrix; // if robotToCameraMat is empty
}

		
/*

namespace ROD{
	//const double PI = CV_PI;//

	// "Rozciąganie histogramu"
	void HistoStretching(int a0, int A, int B, int a1, ::cv::Mat &MImg, ::cv::Mat &NormMImg)
	{
		int my=MImg.rows;
		int mx=MImg.cols;
		int val, nval;

		NormMImg = MImg.clone();
		double wsp =  ((double)(a1 - a0)) / ((double)(B - A));


		for (int j=0; j<mx; j++) { 
			for (int i=0; i<my; i++) {
				val = (int) MImg.at<uchar>(j, i);
				if (val <= A) 
					NormMImg.at<uchar>(j, i) = a0; // min value, e.g. 0
				else
					if (val >=B)
						NormMImg.at<uchar>(j, i) = a1; // max value, e.g. 230
					else { 
						nval = round(a0 + wsp * (val - A)); 
						if (nval  < a0) nval = a0;
						else if (nval > a1) nval = a1;
						NormMImg.at<uchar>(j, i) = nval;
					}
			}
		}

	}

	// Funkcja pomocnicza w wizualizacji - rysowanie prostokąta w obrazie mono
	void DrawBox(::cv::Mat &InImg, int colMin, int colMax, int rowMin, int rowMax, uchar box_col)
	{
		for( int y = rowMin; y < rowMax; y++ )
		{
			InImg.at<uchar>(y, colMin) = box_col;
			InImg.at<uchar>(y, colMax) = box_col;
		}

		for( int x = colMin; x < colMax; x++ )
		{
			InImg.at<uchar>(rowMin, x) = box_col;
			InImg.at<uchar>(rowMax, x) = box_col;
		}
		
	} // Koniec DrawBox

	// Funkcja pomocnicza - wykreśl prostą przechodzącą przez punkt (Cx, Cy) i 
	// zorientowaną pod katem alfa (w radianach) - w obrazie mono
	void DrawLine(double alfa, uchar value, int Cx, int Cy, ::cv::Mat &FCImg)
	{
		int my = FCImg.rows;
		int mx = FCImg.cols;
		double tgalfa = tan(alfa);
		int xr, yr;
		double x, y;
		
		// Generuj wsp. y odpowiadające kolejnym indeksom x
		for (int i=0; i< mx; i++) {
			y = - (tgalfa * (i - Cx) - Cy); // współrzędne obrazu, ale zamień znak Y 
			yr = round(y); 
			if ((yr >= 0) && (yr < my)) {
				FCImg.at<uchar>(yr, i) = value;
			}
		}
		// Generuj wsp. x odpowiadające kolejnym indeksom y 
		for (int i=0; i< my; i++) {
			x = (-(i - Cy))/tgalfa + Cx; // współrzędne obrazu, ale zamień znak Y 
			xr = round(x); 
			if ((xr >= 0) && (xr < mx)) {
				FCImg.at<uchar>(i, xr) = value;
			}
		}
	}

	// Funkcja pomocnicza - tworzy obraz 2D histogramu dla funkcji 1D
	void DrawFunct( ::cv::Mat &histoIR, int histSize, ::cv::Mat &histImg) {				 	
		::cv::normalize(histoIR, histoIR, 0, histImg.rows, CV_MINMAX, CV_32F);
		histImg = ::cv::Scalar::all(255);
		int binW = cvRound((double)histImg.cols/histSize);
		for( int i = 0; i < histSize; i++ ) {
			rectangle( histImg, ::cv::Point(i*binW, histImg.rows), 
					 ::cv::Point((i+1)*binW, histImg.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 );
		}
	}

	*/


/////////////////////////////////////////
// Metody głównej klasy CHuman programu RappAppl
//
/*	
		void Detekcja(char* dirName, char* outDir, const ResultODline *storedRes)
		{
			// Detekcja - zasadnicza funkcja detekcji jabłka i szypułki w obrazach IR 
            // należących do "pakietu obrazów"
			//
            // Dla każdego widoku wywołuje funkcję AnalizaJablka()
            //
            // W wersji off-line funkcji - obrazy są pobierane z plików graficznych w katalogu 
                           
			int numV = prop.getNumViews();
			int numL = prop.getNumLines();

			char fName[128];
			char outName[128];

            // Główna pętla: dla każdej linii i każdego widoku 
			ResultOD jedenWynik;
            for (int i=0; i<numL; i++) //domyślnie linie taśmociągu 
				for (int j=0; j<numV; j++) // domyślnie 8 widoków jabłka
				{
					// dla wersji off-line:
					// określamy nazwy obrazów w plikach
                    sprintf(fName, "%02d/A/%02d", i+1, j+1); // ścieżka i początek nazwy plików wejściowych 
                    sprintf(outName, "%02d_A_%02d", i+1, j+1); // % element nazwy plików z wynikami

					// dla wersji on-line
					// ...

					int offset = i * numL + j;

					

                    // Wywołanie funkcji AnalizaJablka() dla jednego widoku jednego jabłka
                    jedenWynik = AnalizaJablka(dirName, fName, outDir, outName, offset);
					
					// dla wyniku:
					storedRes[j].resultL[i] = jedenWynik;
					
					// Tu należy przechwycić pojedynczy wynik zwracany zmienną jedenWynik
					// ...
					// ...


					if (this->monitorSwitch == 1) // 1: czy ma się zatrzymywać krok-po-kroku
					{
						cout << "Wykonano analizę: linia " << i << "; widok " << j<< endl;
						cv::waitKey(0);
					}

				}
		} // Koniec funkcji Detekcja()
               
  */

          
		
            
        /**
        * @fn Analiza pojedynczego widoku jabłka w celu wykrycia jego połoźenia i szypułki
        
        // PARAMETRY FUNKCJI
        //  fileName - podstawowa nazwa pliku graficznego png;
        //  dirName - nazwa katalogu
        //  outDir - katalog dla wyników
		//  wynik - referncja typu ResultOD dla reprezentacji wyniku

		// WYNIK
		// wynik - opis pojedynczego widoku jednego jabďż˝ka typu ResultOD
      
        // Autor: Wlodzimierz Kasprzak  & Jan Figat
        // Modyfikacja: 4-11-2014
        // 
		

        ResultOD AnalizaJablka(char* dirName, char* fileName, char* ouDir, char* ouName, int offset)
		{
			// Ustaw obiekty - PARAMETRY analizy obrazu
			Parameters param = this->param;
            Properties prop = this->prop;
            UserParameters uparam = this->userparam;

			// Przełączniki - wizualizacja i monitorowanie wyników częściowych
			int VISUAL = this->visualSwitch; // 1: czy ma prezentowaďż˝ okna on-line
			int MONITOR = this->monitorSwitch; // 1: czy ma siďż˝ zatrzymywaďż˝ krok-po-kroku
			int LOGS = this->logSwitch; // 1: utrwalenie wynikďż˝w w plikach dla ilustracji dziaďż˝ania funkcji


			// Parametry szczegółowe
			int DIRS = param.directions; // 720: podział kąta pełnego na 720 części

			// Nazwy plików
			char fIrName[128];
			char outName[128];
			// Pomiar czasu
			double t0, elapsed;
			time_t start, end;
			// Zakres kątowy przeszukiwania konturu i wnętrza
			cv::Mat wycinekPoIr = cv::Mat::zeros(2,1,CV_32SC1);
			cv::Mat wycinek = cv::Mat::zeros(3,1,CV_64FC1);
			
			// Zaczynamy
			
			if (MONITOR == 1) std::cout<< "AnalizaJablka() start\n"; 
			
			/////////////////////////////////
			// Krok A1. Ścieżka dostępu do obrazu, jego wczytanie i normalizacja
			//
			t0=clock();		// zliczanie czasu
			start=time(0);
			//
			// 1.1 Wczytaj obraz IR 
			//
			sprintf(fIrName, "%s/%s_0.png\0", dirName, fileName); 
			
			if (MONITOR == 1) 
				std::cout<< fIrName << std::endl; 
			
			cv::Mat IRImg;
			// TEST only:
			//IRImg = cv::imread("C:/SORTER/img/Database/01_Diffuse/01/A/02_0.png");

			IRImg = ::cv::imread(fIrName);
	

			if(! IRImg.data ){							///sprawdzenie czy plik istnieje
			    cout<<"\n"<<fIrName<<" -- IR"<<endl;
			        throw  "Nie można otworzyć pliku" ;
			}
			if (MONITOR == 1) 
				std::cout << "L kanałow IRImg: " << IRImg.channels() << std::endl; 
			
			if (VISUAL== 1) { 	
				::cv::imshow("A1.1 Obraz IR", IRImg); 
			}

			int gy = IRImg.rows;
			int gx = IRImg.cols;
			
			// Obraz IR posiada niepotrzebnie 3 płaty
			vector<cv::Mat> IRplanes; // Vector to klasa szablonowa
			::cv::split(IRImg, IRplanes); // Rozdziel obraz na 3 płaty 

			//
			//1.2. Normalizacja histogramu - rozciąganie lub przeskalowanie
			//
			int histSize = 256;
			::cv::Mat histoIR;
			::cv::Mat histImage = ::cv::Mat::ones(256, 256, CV_8U)*255;
			
			//Wyznacz histogram obrazu IR (tylko w trybie wizualizacji)
			if (VISUAL== 1) {
				 ::cv::calcHist(&IRplanes[0], 1, 0, ::cv::Mat(), histoIR, 1, &histSize, 0);
				 
				 ::cv::normalize(histoIR, histoIR, 0, histImage.rows, CV_MINMAX, CV_32F);
				 histImage = ::cv::Scalar::all(255);
				 int binW = cvRound((double)histImage.cols/histSize);
				 for( int i = 0; i < histSize; i++ ) 
					 rectangle( histImage, ::cv::Point(i*binW, histImage.rows), 
					 ::cv::Point((i+1)*binW, histImage.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 ); 
				::cv::imshow("A1.1 Histogram obrazu IR", histImage); 
			}

			// Normalizacja histogramu
			// W oryginale: było rozciąganie histogramu obrazu
			// z parametrami [0.05 0.95] i odwzorowaniem [0.0 0.90]*255
			
			double minVal, maxVal;			
			double alpha ; //
			// Rozciąganie/przeskalowanie dla obrazu IR - 
			::cv::Mat NormIRImg;
			::cv::minMaxIdx(IRplanes[0], &minVal, &maxVal, NULL, NULL, ::cv::noArray());
			alpha = 230.0/ maxVal;
			alpha = 0.9 * maxVal;
			if (MONITOR == 1)
				cout<< "MaxVal for IR: "<< maxVal << " alpha: " << alpha << endl;

			if (alpha > 200)
				alpha = 200;
			
			// Rozciąganie
			ROD::HistoStretching(0, 20, (int)alpha, 230, IRplanes[0], NormIRImg);
			// O ile IRImg ma 3 płaty, to NormIRImg bedzie miał już tylko 1
			
			// Lub przeskalowanie:
			// IRplanes[0].convertTo(NormIRImg, CV_8U, alpha);
			
			// Wizualizacja wyniku
			if (VISUAL== 1) { 
				::cv::imshow("A1.2 Norm IR", NormIRImg); 
			}
			// Utrwalenie wyniku
			if (LOGS == 1)
			{ 
				sprintf(outName, "%s/NormIR_%s.png", ouDir, ouName); 
				cv::imwrite(outName, NormIRImg);
			}


			//1.3 Wyznacz histogram po przeskalowaniu (tylko w trybie wizualizacji)
			if (VISUAL== 1) { 
				 ::cv::calcHist(&NormIRImg, 1, 0, ::cv::Mat(), histoIR, 1, &histSize, 0);
				 
				 ::cv::normalize(histoIR, histoIR, 0, histImage.rows, CV_MINMAX, CV_32F);
				 histImage = ::cv::Scalar::all(255);
				 int binW = cvRound((double)histImage.cols/histSize);
				 for( int i = 0; i < histSize; i++ ) 
					 rectangle( histImage, ::cv::Point(i*binW, histImage.rows), 
					 ::cv::Point((i+1)*binW, histImage.rows - cvRound(histoIR.at<float>(i))),
					 ::cv::Scalar::all(0), -1, 8, 0 ); 
				::cv::imshow("A1.3 Histogram IR po skalowaniu", histImage); 	
				
			}

			// Koniec pomiaru czasu dla kroku A1
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 1: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
			// Koniec A1
			////////////////////////////
	

			////////////////////////////
			// A2. Obraz krawędziowy w IR ( w zasadzie będzie potrzebny jedynie dla szypułki wewnętrznej) 
			//
			t0=clock();		// zliczanie czasu
			start=time(0);
			
			// Canny
			//::cv::Canny(NormMonoImg, EdgeMImg, param.edgeThresh *255, param.edgeThresh*2, 3, false);
			// Jednak nie daje on kierunku krawędzi dla elementu krawędziowego
			// Dlatego potrzebna jest własna funkcja

			// Operator krawędziowy w IR
			::cv::Mat EdgeIRMImg, EdgeIRDirImg;
			// Funkcja globalna
			// Operator krawędziowy i pocienianie z progiem względnym, np. edgeThresh=0.2:
			EdgeDetection( param.lowThresh, param.edgeThresh, NormIRImg, EdgeIRMImg, EdgeIRDirImg);
			
			if (VISUAL== 1) { 
				::cv::Mat ShowImg = ::cv::Mat::zeros(20, 20, CV_8U);
				NormIRImg.copyTo(ShowImg, EdgeIRMImg);
				cv::imshow("A2. Krawędzie w IR", ShowImg);
				// Dokumentacja wyniku
				if (LOGS == 1)
				{ 
					sprintf(outName, "%s/EdgeIR_%s.png", ouDir, ouName); 
					::cv::imwrite(outName, EdgeIRMImg);
				}
				ShowImg.release();
			}   

			// Koniec pomiaru czasu dla kroku 2
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 2: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
			// Koniec A2
			///////////////////////////////


			///////////////////////////////
			// A3. Wyznaczenie prostokątnego obszaru zainteresowania ROI IR
			// 
			
			t0=clock();		// zliczanie czasu
			start=time(0);
		
			//3.1 ROI w obrazie IR
			int IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax;
			int bbox[4];
			
			DetectROI2(NormIRImg, param.cornerMax, param.cornerMin, bbox);
			
			IRcolumnMin=bbox[0];
			IRcolumnMax=bbox[1];
			IRrowMin=bbox[2];
			IRrowMax=bbox[3];

			if (MONITOR == 1)
				cout<<"IR bbox:" << IRcolumnMin << ", "<< IRcolumnMax << ", " << IRrowMin << ", " << IRrowMax <<endl;

			// Pokaż obszar ROI w obrazie IR
			if(VISUAL==1)
			{
				::cv::Mat ShowROI_IRImg = NormIRImg.clone();
				ROD::DrawBox(ShowROI_IRImg, IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 255);
				cv::imshow("A3.1 ROI IR", ShowROI_IRImg);
				// Dokumentacja wyniku
				if (LOGS == 1)
				{ 
					sprintf(outName, "%s/ROIIR_%s.png", ouDir, ouName); 
					::cv::imwrite(outName, ShowROI_IRImg);
				}
				ShowROI_IRImg.release();
			}
			

			/*
			// 3.2 ? Sprawdzenie detekcji konturu operatorami morfologicznymi 
			::cv::Mat dst2;
			::cv::Rect rect;
			::cv::Mat dst = NormMonoImg.clone();
			floodFill(dst, ::cv::Point(120, 150), ::cv::Scalar(230), &rect, ::cv::Scalar(3), ::cv::Scalar(3));

			//ErosionEllipse( NormMonoImg,  dst);
			// 3.3
			DilationEllipse( dst,  dst2);
			// Wizualizacja wyniku operacji morfologicznych
			if (VISUAL == 1) {
				cv::imshow( "A3.3 Fill", dst );
				cv::imshow( "A3.4 Dilation", dst2 );
			}
			

			// Koniec pomiaru czasu dla kroku 3
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 3: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;
	
			
			/////////////////////////
			// A4. Znajdź dokładny brzeg obiektu (zamknięty kontur) w obrazie IR
			//
			// Kontur w obrazie IR
			int Cirx, Ciry; // środek masy konturu w obrazie IR
			::cv::Mat IrKontur = ::cv::Mat::zeros(DIRS, 2, CV_64FC1); // wektor współrzędnych punktów konturu
			::cv::Mat IrKontur1D = ::cv::Mat::zeros(DIRS, 1, CV_64FC1); // odległości punktów konturu od środka masy
			::cv::Mat IrGrad1D = ::cv::Mat::zeros(DIRS, 1, CV_64FC1); // gradient funkcji odległości
			//

			t0=clock();             // zliczanie czasu
			start=time(0);
			   
			int InitCy, InitCx; // środek obszaru ROI w obrazie IR
			double dIRcolumnMin, dIRcolumnMax, dIRrowMin, dIRrowMax;
			int radius; // dla końcowego promienia konturu
			int minRadius; // ograniczenie w procesie poszukiwania konturu
			
			// IRrowMin, IRrowMax, IRcolumnMin, IRcolumnMax to spodziewane ROI
			dIRcolumnMin = IRcolumnMin;
			dIRcolumnMax = IRcolumnMax;
			dIRrowMin = IRrowMin;
			dIRrowMax = IRrowMax;
			InitCy = round((dIRrowMin + dIRrowMax)/2);
			InitCx = round((dIRcolumnMin + dIRcolumnMax)/2);
			minRadius = round(((dIRcolumnMax - dIRcolumnMin) + (dIRrowMax - dIRrowMin))/ 4.0);

			// Funkcja szukania konturu w obrazie IR
			// Wynik zwracany jest w postaci zmiennych:
			// Cirx, Ciry, IrKontur, IrKont1D, GradIrKont1D
			// oryg. parametry = 0.2, 0.3
			KonturIR(0.15, 0.20, DIRS, InitCx, InitCy, minRadius, NormIRImg, 
				 Cirx, Ciry, IrKontur, IrKontur1D,  IrGrad1D);
						
			// Teraz właściwe BBox odpowiada ograniczeniom konturu IR:
			cv::minMaxLoc(IrKontur.col(0), &dIRcolumnMin, &dIRcolumnMax,0,0,cv::Mat());
			if (dIRcolumnMin <0)	dIRcolumnMin =0;
			if (dIRcolumnMax >= gx) dIRcolumnMax = gx-1;
			
			cv::minMaxLoc(IrKontur.col(1),&dIRrowMin,&dIRrowMax,0,0,cv::Mat());
			if (dIRrowMin <0)  dIRrowMin =0;
			if (dIRrowMax >= gy)  dIRrowMax = gy-1;
			
			// Zamiana na wartości całkowite
			IRcolumnMin = int(dIRcolumnMin);
			IRcolumnMax = int(dIRcolumnMax);
			IRrowMin = int(dIRrowMin);
			IRrowMax = int(dIRrowMax);

			// Wizualizacja konturu w IR w kolejnym kroku 5 

			// Koniec pomiaru czasu dla kroku 4
			end=time(0);
			elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
			cout << "Czas kroku 4: " << endl;
			cout << elapsed<< " ms" <<  endl;
			cout << end-start << " s" <<  endl;

			// Koniec kroku 4
			////////////////////////

			///////////////
			// 5. Detekcja szypułki odstającej w obrazie IR
			
			//
			t0=clock();         // początek zliczania czasu
			start=time(0);

			// 5.1 Analiza funkcji 1D konturu w obrazie IR
			cv::Mat Wycinek = cv::Mat::zeros(2,1,CV_32SC1);
			// Szukamy w całym zakresie konturu
			Wycinek.at<int>(0,0)= 0; 
			Wycinek.at<int>(1,0)= DIRS-1;

			// Funkcja zwraca wynik w postaci zmiennych globalnych:
			// [jestOgonIr, pozIrOgon, alfaIr, CirxNew, CiryNew, MaskaIrOgon, BBoxIr]
			// warto to zmienić
			
			int CirxN, CiryN;

			//Zmieniona wartość - oryginalnie 120.0 (wspPIK)
			::cv::Mat BBoxIr = OgonekIR(gy, gx, 5, Cirx, Ciry, Wycinek, IrKontur, IrKontur1D, IrGrad1D,
				CirxN, CiryN);
			
			// Po ewentualnym uwzględnieniu szypułki odstającej
			// zmienił się BBox obszaru samego jablka
				IRcolumnMin = BBoxIr.at<int>(0,0); // xmin
				IRcolumnMax = BBoxIr.at<int>(2,0); // xmax
				IRrowMin = BBoxIr.at<int>(1,0); // ymin
				IRrowMax = BBoxIr.at<int>(3,0); // ymax
			// Promień konturu:
				radius = BBoxIr.at<int>(4,0); // promień

			// Utwórz prostokatny ROI szypułki w IR
			cv::Mat roiSzypIr = cv::Mat::zeros(4,1,CV_32SC1);
			if (jestOgonIr == 1)
			{
				roiSzypIr.at<int>(0,0) =(pozIrOgon.at<int>(0,0) - 20); // xmin
				if (roiSzypIr.at<int>(0,0) < 0) 
					roiSzypIr.at<int>(0,0) = 0;
				roiSzypIr.at<int>(1,0) =(pozIrOgon.at<int>(1,0) - 20); // ymin
				if (roiSzypIr.at<int>(1,0) < 0) 
					roiSzypIr.at<int>(1,0) = 0;
				roiSzypIr.at<int>(2,0) =(pozIrOgon.at<int>(0,0) + 20); // xmax
				if (roiSzypIr.at<int>(2,0) >= gx) 
					roiSzypIr.at<int>(2,0) = gx-1;
				roiSzypIr.at<int>(3,0) =(pozIrOgon.at<int>(1,0) + 20); // ymax
				if (roiSzypIr.at<int>(3,0) >= gy) 
					roiSzypIr.at<int>(3,0) = gy-1;

				if (MONITOR == 1)
					cout <<"IrOgon: x="<< pozIrOgon.at<int>(0,0) <<", y=" ;
					cout << pozIrOgon.at<int>(1,0) << endl;
				
				
			}
			else
			{
				roiSzypIr.release(); 
			}

			// Wizualizacja konturu i ogonka w obrazie IR
			cv::Mat MonoE3Img;
			cv::Mat ShowMonoE3Img;
			std::vector<cv::Mat> MonoE3Img_planes(3);
			if (VISUAL == 1)
			{
				// Wizualizacja konturu w obrazie
				int x,y;
				//cv::split(F1ColorImg,MonoE3Img_planes);

				MonoE3Img_planes[0] = NormIRImg.clone();
				MonoE3Img_planes[1] = NormIRImg.clone();
				MonoE3Img_planes[2] = NormIRImg.clone();
				
			    for (int i=0;i<DIRS;i++)
			    {
			        y = int(IrKontur.at<double>(i,1) );
			        x = int(IrKontur.at<double>(i,0) );

			        if (MaskaIrOgon.at<uchar>(i,0) == 1)
			        {
			        	// ogonek
			        	MonoE3Img_planes[0].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[1].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[2].at<uchar>(y,x) = 255;
			        }
			        else
			        {
			            // kontur
			        	MonoE3Img_planes[0].at<uchar>(y,x) = 255;
			        	MonoE3Img_planes[1].at<uchar>(y,x) = 100;
			        	MonoE3Img_planes[2].at<uchar>(y,x) = 0;
			        }
			    }
				// Ewentualnie dorysuj ROI ogonka
				if (jestOgonIr == 1) {
					ROD::DrawBox(MonoE3Img_planes[0], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),100);
					ROD::DrawBox(MonoE3Img_planes[1], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),100);
					ROD::DrawBox(MonoE3Img_planes[2], roiSzypIr.at<int>(0,0), roiSzypIr.at<int>(2,0),roiSzypIr.at<int>(1,0),roiSzypIr.at<int>(3,0),255);

				// Dorysuj oś
					DrawLine(alfaIr.at<double>(0,0), 255, CirxN, CiryN, MonoE3Img_planes[0]); // red plane
					DrawLine(alfaIr.at<double>(1,0), 255, CirxN, CiryN, MonoE3Img_planes[0]); // red plane
					DrawLine(alfaIr.at<double>(0,0), 150, CirxN, CiryN, MonoE3Img_planes[1]); // g plane
					DrawLine(alfaIr.at<double>(1,0), 150, CirxN, CiryN, MonoE3Img_planes[1]); // g plane
					DrawLine(alfaIr.at<double>(0,0), 50, CirxN, CiryN, MonoE3Img_planes[2]); // b plane
					DrawLine(alfaIr.at<double>(1,0), 50, CirxN, CiryN, MonoE3Img_planes[2]); // b plane
				}

				// Narysuj prostokąt obejmujący obszar jabłka (bez szypułki) 
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 0);
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 100);
				ROD::DrawBox(MonoE3Img_planes[0], IRcolumnMin, IRcolumnMax, IRrowMin, IRrowMax, 255);

				// Dorysuj środek
				ROD::DrawBox(MonoE3Img_planes[0], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 255);
				ROD::DrawBox(MonoE3Img_planes[1], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 0);
				ROD::DrawBox(MonoE3Img_planes[2], CirxN - 2, CirxN + 2, CiryN - 2, CiryN + 2, 0);

				//
			    cv::merge(MonoE3Img_planes, ShowMonoE3Img );
			    cv::imshow("A5.1 Kontur i szypulka w IR", ShowMonoE3Img );
			    if (LOGS == 1)
			    {
			    	sprintf(outName, "%s/KonturIRSzyp_%s.png", ouDir, ouName);
			    	cv::imwrite(outName, ShowMonoE3Img );
			    }
			    
				ShowMonoE3Img.release();
				MonoE3Img_planes[0].release();
				MonoE3Img_planes[1].release();
				MonoE3Img_planes[2].release();

			}

			// 

		    //
		    // 5.2 Detekcja szypułki wewnętrznej lub granicznej
		    // - na podstawie obrazu krawędziowego i obrazu IR
			// ...
			int jestSzyp = 0; // inicjalizacja

		    
			// Koniec pomiaru czasu dla kroku 5
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 5: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;
			//
			//////////////////////


			//////////////////////
		    // 6. Analiza 2D obrazu IR (AW) - detekcja szypułki odstającej
		    //
		    t0=clock();             // zliczanie czasu
		    start=time(0);

		    // Koniec pomiaru czasu dla kroku 6
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 6: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;

			///////////////////
		    //% 7. Detekcja zagłębień w Mono (AW) - detekcja szypułki wewnętrznej / kielicha 
		    t0=clock();             // zliczanie czasu
		    start=time(0);

		    // Koniec pomiaru czasu dla kroku 7
		    end=time(0);
		    elapsed= (double)(clock() - t0) / CLOCKS_PER_SEC;
		    cout << "Czas kroku 7: " << endl;
		    cout << elapsed<< " ms" <<  endl;
		    cout << end-start << " s" <<  endl;


			/////////////////////
			// 8. Spakuj wyniki
			if (VISUAL == 1)
				cv::destroyAllWindows();
			
			ROD::ResultOD result = ROD::ResultOD();
			// TO DO: trzeba zapakowac wyniki do struktury zwracanej
			// ...
			result.cx = CirxN; // Uwaga: w układzie obrazu !
			result.cy = CiryN; //  - " -
			result.radius = radius;
			result.jestOgon = jestOgonIr; // odstająca szypułka
			result.jestSzyp = jestSzyp; // wewnętrzna 
			result.ogonCx = pozIrOgon.at<int>(0,0); // położenie nasady szypułki
			result.ogonCy = pozIrOgon.at<int>(1,0);
			result.bBox[0] = IRcolumnMin; // minX - prostokąt obejmujący jabłko
			result.bBox[2] = IRcolumnMax; // maxX
			result.bBox[1] = IRrowMin; // minY
			result.bBox[3] = IRrowMax; // maxY

			return result;
			}
	//////////////////////////////////////////
		
	////////////////////////////////////////////
	*/
