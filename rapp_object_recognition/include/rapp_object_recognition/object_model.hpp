#ifndef OBJECT_MODEL
#define OBJECT_MODEL

#include <opencv2/opencv.hpp>

class ObjectModel {
public:
	/// Object name
	std::string name;
	
	/// Object training image
	cv::Mat image;
	
	/// Keypoints detected in training image
	std::vector<cv::KeyPoint> keypoints;
	
	/// Descritors calculated for detected keypoints
	cv::Mat descriptors;
	
	/**
	 * Load model stored on disk
	 * 
	 * \param [in] base_path Root folder 
	 * \param [in] name Model name
	 * 
	 * \return Operation status
	 */
	bool load(const std::string & base_path, const std::string & mname) {
		cv::FileStorage fs(base_path + mname + ".yml", cv::FileStorage::READ);
		if (!fs.isOpened()) return false;
	
		std::string fname;
		cv::FileNode kptFileNode = fs["keypoints"];
		cv::read( kptFileNode, keypoints );
		fs["descriptors"] >> descriptors;
		fs["fname"] >> fname;
		fs.release();
		
		image = cv::imread(base_path + fname, -1);
		
		name=mname;
		
		return true;
	}
	 
	 /**
	  * Save model on disk
	  * 
	 * \param [in] base_path Root folder 
	 * \param [in] name Model name
	 * 
	 * \return Operation status
	 */
	bool save(const std::string & base_path, const std::string & name) {
		cv::imwrite(base_path + name + ".png", image);
		
		auto fs_path = base_path + name + ".yml";
		cv::FileStorage fs(fs_path, cv::FileStorage::WRITE);
		cv::write( fs, "keypoints", keypoints );
		cv::write( fs, "descriptors", descriptors);
		cv::write( fs, "fname", name + ".png");
		fs.release();
	}
};

#endif /* OBJECT_MODEL */

