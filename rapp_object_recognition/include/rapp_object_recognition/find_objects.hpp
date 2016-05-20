#include <vector>
#include <string>

#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;

class FindObjects {
public:
	FindObjects() : matcher(NORM_HAMMING, true) {
	}

	int findObjects(const std::string & user, const std::string & fname, const std::vector<std::string> & names, const std::vector<std::string> & files, unsigned int limit, 
	                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores);
	
	void learnObject(const std::string & user, const std::string & fname, const std::string & name, int & result);
	
	void clearModels(const std::string & user);
	
	void loadModel(const std::string & user, const std::string & name);
	
	void loadModels(const std::string & user, const std::vector<std::string> & names, int & result);

protected:
	bool loadImage(const std::string filename_, cv::Mat & image_);
	
	bool extractFeatures(const cv::Mat image_, std::vector<KeyPoint> & keypoints_, cv::Mat & descriptors_);

	void loadModels(std::vector<std::string> names_, std::vector<std::string> files_);
	
	void storeObjectHypothesis(std::string name_, cv::Point2f center_, std::vector<cv::Point2f> corners_, double score_, unsigned int limit_);

private:

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
	
	/// Keypoint detector.
	cv::OrbFeatureDetector detector;

	/// Feature descriptor.
	cv::OrbDescriptorExtractor extractor;

	// Matcher.
	cv::BFMatcher matcher;
};
