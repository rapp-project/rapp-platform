#include <vector>
#include <string>

#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;

/**
 * Textured objects detector.
 * Extracts feature points from image and compares them to those stored 
 * in model files. Allows to learn new models. At recognition step all
 * known objects or only selected subset can be used.
 */
class FindObjects {
public:
	/**
	 * Default constructor.
	 * 
	 * \param silent Suppress console output
	 */
	FindObjects(bool silent = true);

	/**
	 * Find objects using curretly selected subset of models.
	 * 
	 * \param [in] user Username (to look for the models)
	 * \param [in] fname Absolute path to the query image
	 * \param [in] limit Limit searching to N best matches
	 * \param [out] found_names Names of found objects
	 * \param [out] found_centers Centers of found objects in query image
	 * \param [out] found_scores Matching score for found objects (can be used as recognition certainity)
	 *
	 * \return Operation status; 0 - ok, -1 - no models loaded, -2 - no image to analyse 
	 */
	int findObjects(const std::string & user, const std::string & fname, unsigned int limit, 
	                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores);
	
	/**
	 * Learn new objects model.
	 * 
	 * \param [in] user Username (to store the learnt model)
	 * \param [in] fname Absolute path to the object picture
	 * \param [in] name Name of the object
	 * 
	 * \return Operation status; 0 - ok, -2 - no image to analyse 
	 */
	int learnObject(const std::string & user, const std::string & fname, const std::string & name);
	
	/**
	 * Clear list of models used for recognition.
	 * Only operating list of models is affected, all learnt models are
	 * kept intact.
	 * 
	 * \return Operation status
	 */
	bool clearModels(const std::string & user);
	
	/**
	 * Load model to operating list for recognition.
	 * 
	 * \param [in] user Username (to look for the model)
	 * \param [in] name Name of the model
	 * 
	 * \return Operation status
	 */
	bool loadModel(const std::string & user, const std::string & name);
	
	/**
	 * Load multiple models to operating list for recognition.
	 * 
	 * \param [in] user Username (to look for the models)
	 * \param [in] names List of model names to load
	 * \param [out] result Overall operation status
	 * 
	 * \return Model loading status for each object
	 */
	std::map<std::string, bool> loadModels(const std::string & user, const std::vector<std::string> & names, int & result);

protected:
	/**
	 *  Load image.
	 * 
	 * \param [in] filename_ Absolute path to the file
	 * \param [out] image_ Target image to store loaded picture
	 * 
	 * \return Operation status
	 */
	bool loadImage(const std::string & filename_, cv::Mat & image_);
	
	/**
	 * Extract keypoints and features from given image.
	 * 
	 * \param [in] image_ Query image
	 * \param [out] keypoints_ Found keypoints location
	 * \param [out] descriptors_ Descriptors calculated for found keypoints
	 * \param [in] grid Use grid detector adaptor
	 * \param [in] mask Mask for keypoint detection
	 * 
	 * \return Operation status
	 */
	bool extractFeatures(const cv::Mat image_, std::vector<KeyPoint> & keypoints_, cv::Mat & descriptors_, bool grid = false, cv::Mat mask_ = cv::Mat());

	/**
	 * Store given object hypothesis as recognized object.
	 * 
	 * \param [in] name_ Object name
	 * \param [in] center_ Object center
	 * \param [in] corners_ Objects boundging-box corners
	 * \param [in] score_ Recognition score
	 * \param [in] limit_ Remove the worst hypotheses if limit reached
	 */
	void storeObjectHypothesis(std::string name_, cv::Point2f center_, std::vector<cv::Point2f> corners_, double score_, unsigned int limit_);

private:

	// ******************************** MODELS ********************************
	
	/// Vector of images constituting the consecutive models.
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
	cv::Ptr<cv::OrbFeatureDetector> detector;

	/// Feature descriptor.
	cv::Ptr<cv::OrbDescriptorExtractor> extractor;

	/// Grid keypoint detector adapter
	cv::Ptr<cv::GridAdaptedFeatureDetector> grid_detector;

	/// Feature matcher.
	cv::Ptr<cv::DescriptorMatcher> matcher;
	
	/// Silent output flag
	bool silent_;
};
