// Human.h
// Deklaracja klasy CHuman

#ifndef HUMAN
#define HUMAN

#include "rapp_visual_localization/Parameters.hpp"
#include "rapp_visual_localization/Properties.hpp"
#include "rapp_visual_localization/UserParameters.hpp"
#include "rapp_visual_localization/ResultOD.hpp"

#include <time.h>

#include <opencv2/opencv.hpp>

using namespace std;


/////////////////////////////////////////
// Klasa
//
	class CHuman{
	public:
		// obiekt pomocniczy
		// ...

	private:
		// pola : dane we/wy i parametry analizy
		char  *streamIN; // katalog dla procesu modelowania
		char *streamOUT; // katalog dla wyników procesu modelowania
        CProperties prop;
        CParameters param;
        CUserParameters userparam;
        CResultOD *pattern; // alternatywne modele sylwetki

		// pola: przełączniki wizualizacji i monitorowania
		int visualSwitch; // 1: czy ma prezentować okna on-line
		int monitorSwitch; // 1: czy ma się zatrzymywać krok-po-kroku
		int logSwitch; // 1: czy zapisywać wyniki do plików 

		// ****************************** HYPOTHESES ******************************

		/// Vector containing names of recognized objects.
		std::vector<std::string> recognized_names;
		//std::vector< std::vector<std::string> > recognized_names_vec;

		/// Vector containing centers of recognized objects (image coordinates).
		std::vector<cv::Point2f> recognized_centers;
		//std::vector< std::vector<cv::Point2f> > recognized_centers_vec;

		/// Vector containing quadruples of corners of recognized objects (image coordinates).
		std::vector< std::vector< cv::Point2f > > recognized_corners;
		//std::vector< std::vector< std::vector< cv::Point2f > > > recognized_corners_vec;

		/// Vector containing scores of recognized objects.
		std::vector<double> recognized_scores;
		//std::vector< std::vector<double> >recognized_scores_vec;

		/// Vector containing information about projection (Valid/Rejected)
		std::vector< bool > valid_projection;

		// Vector containing information about frame id, on which a model was detected
		std::vector<int> recognized_frameId;


		// ******************************** MODELS ********************************

		/// Vector of images constituting the consecutive models.
		std::vector< cv::Mat > models_imgs;
		std::vector< std::vector<cv::Rect> > models_rois;

		/// Vector of keypoints of consecutive models.
		std::vector< std::vector< cv::KeyPoint > > models_keypoints;

		/// Vector of descriptors of consecutive models.
		std::vector< cv::Mat > models_descriptors;
		//std::vector< std::vector< cv::Mat > > models_descriptors_vec;

		/// Vector of names of consecutive models.
		std::vector< std::vector< std::string > > models_names_vec;
		std::vector< std::string > models_names;

		// ******************************** FILTER ********************************
		bool initialized;
		bool predict;
		double human_position_x, human_position_y;
		bool new_data;

		// **************************** OTHER VARIABLES ****************************

		/// Keypoint detector and extractor
		int 	nfeatures;
		float 	scaleFactor;
		int 	nlevels;
		int 	edgeThreshold;
		int 	firstLevel;
		int 	WTA_K;
		int 	scoreType;
		int 	patchSize;
		int 	fastThreshold;
		/// Keypoint detector.
		cv::Ptr<cv::OrbFeatureDetector> detector;
		/// Feature descriptor.
		cv::Ptr<cv::OrbDescriptorExtractor> extractor;
		// Matcher.
		//cv::BFMatcher matcher(cv::NORM_HAMMING, true); // Brute-Force matcher // For the binary descriptors
		cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;


		// ********************  Functions for keypoint model *********************
		// features extraction
		bool extractFeatures(const cv::Mat image_, std::vector<cv::KeyPoint> & keypoints_, cv::Mat & descriptors_);

		// models creation
		void createModelsFromImages(std::vector<cv::Mat> & images_, std::vector< std::vector<cv::Rect> > & rois_,
			std::vector< std::vector<std::string> > & names_,
			std::string output_model_path, std::string model_file_name // sciezka do pliku z modelem
			);

		// adds hypothesis
		void storeObjectHypothesis(std::string name_, cv::Point2f center_, std::vector<cv::Point2f> corners_, double score_, int frameId_, unsigned int limit_,
			std::vector<std::string> &recognized_names_, std::vector<cv::Point2f> &recognized_centers_,
			std::vector<std::vector<cv::Point2f> > &recognized_corners_, std::vector<double> &recognized_scores_,
			std::vector<int> & recognized_frameId_);

		// matching with a cross check
		void crossCheckMatching(cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
			const cv::Mat& descriptors1, const cv::Mat& descriptors2,
			std::vector<cv::DMatch>& filteredMatches12, int knn = 1);

		// Find knn nearest neighbors
		void multiCheckMatching(const cv::Mat& objectDescriptors, const cv::Mat& sceneDescriptors,
			std::vector<std::vector<cv::DMatch> >& matches, int knn = 2);

		// Process Nearest Neighbor Distance Ratio
		int processNNDR(std::vector<std::vector<cv::DMatch> >& matches, std::vector<cv::KeyPoint>& objectKeypoints, std::vector<cv::KeyPoint>& sceneKeypoints,
			cv::Mat & img_, int img_id, // information about image frame
			std::string & model_name_, // model name
			unsigned int minInliers, // minimal number of inliers (>=4)
			cv::Rect models_rois,
			unsigned int limit, // Limit the size of the received vectors
			std::vector<std::string> & found_names, std::vector<cv::Point2f> & found_centers, std::vector<std::vector<cv::Point2f> > &recognized_corners_, std::vector<double> & found_scores, std::vector<int> & recognized_frameId_);// vectors with a detection results

		// detect known objects
		int findObjects(const bool isMultiple, cv::Mat & img_, int img_id, std::vector< std::vector< std::string > > & models_names_, std::vector< std::vector<cv::Rect> > models_rois_, unsigned int limit,
			std::vector<std::string> & found_names, std::vector<cv::Point2f> & found_centers, std::vector<std::vector<cv::Point2f> > &found_corners, std::vector<double> & found_scores, std::vector<int> & recognized_frameId_);

		// clear the data
		void clearHypRes(){
			recognized_names.clear();
			recognized_centers.clear();
			recognized_corners.clear();
			recognized_scores.clear();
			valid_projection.clear();
			recognized_frameId.clear();
		}
	
	public:

		// loading the model
		void loadModelFromFile(std::string path_, std::string model_file_name);

	   	CHuman(char *in, char* out, CProperties pr, CParameters pa, CUserParameters upa,
			int vis=0, int monit=0, int log=0)
		{
			this->streamIN = in;
			this->streamOUT = out;
            this->prop = pr;
            this->param = pa;
            this->userparam = upa;
            this->pattern = new CResultOD[upa.getViews()];
			
			this->visualSwitch = vis; // 1: czy ma prezentować okna on-line
			this->monitorSwitch = monit; // 1: czy ma drukować komentarze
			this->logSwitch = log; // 1: czy ma utrwalać obrazy wynikowe na dysku
		}
		
		//
		//CHuman()
		//{
		//}
		
		/**
		* @brief Domyślny konstruktor
		*/
		CHuman(void);

		//// temporary functions
		void createModel(char* directoryIN, char* directoryOUT, char* modelFilenameXml,
			bool isModel=false, int visualswitch=0, int logswitch=0, int monitorswitch=0);

		::cv::Rect detectPerson(::cv::Mat obraz);
		////

		/**
		* @brief Generacja wzorca
		* @param models_imgs Wektor zawierajacy obrazy
		* @param models_rois Wektor zawierajacy wspołrzedne zaznaczonych obiektow
		* @param models_names_vec Wektor zawierajacy nazwy modeli
		*/
		void createModel(std::vector<cv::Mat> & models_imgs,// wektor zawierający obrazy
			std::vector< std::vector<cv::Rect> > & models_rois,// wektor zawierający współrzędne zaznaczonych obiektów
			std::vector< std::vector<std::string> > & models_names_vec,// wektor zawierający nazwy modeli
			std::string output_model_path, std::string model_file_name // sciezka do pliku z modelem
			);

		std::vector< ::cv::Rect > detectPerson(::cv::Mat obraz,
			std::string model_path,
			std::string model_name,
			int mode = 0 // 0 - model and HOG pedestrian, 1 - only model, 2 - only HOG pedestrian
			);

		// human in the camera coordinate system
		cv::Mat computeHumanToCamera(double camera_matrix[][3],
			double TheoreticalHeight,
			double TheoreticalWidth,
			//std::vector< cv::Point2f > humanRect,
			cv::Rect humanRect,
			cv::Mat robotToCameraMat);
		

	}; // Koniec definicji klasy Analysis
	////////////////////////////////////////////
	

#endif
