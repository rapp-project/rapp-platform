/******************************************************************************
Copyright 2015 RAPP

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

written by Jan Figat
******************************************************************************/

#include <face_recognition/face_recognizer.h>


/** 
 * @brief Default constructor
 */
FaceRecognizer::FaceRecognizer(void)
{
}

    /**
     * @brief   Loads an image from a file URL
     * @param   file_name [std::string] The image's file URL
     * @return  [cv::Mat] The image in OpenCV representation
     */
cv::Mat FaceRecognizer::loadImage(std::string file_name)
{
	cv::Mat input_img;
	// Must check if file exists
	input_img = cv::imread(file_name);
	return input_img;
}


    /**
     * @brief   Read CSV file from the URL
     * @param   filename [const std::string&] The input CSV file's URL
     * @param   images [std::vector<cv::Mat>&] The vector which will be filled with face's images
     * @param   labels [std::vector<int>&] The vector which will be filled with face's labels
     * @param   separator [char] The separator in CSV file (between image and label)
     */
void FaceRecognizer::read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, char separator)
{
	std::ifstream file(filename.c_str(), std::ifstream::in);
	if (!file) {
		std::string error_message = "No valid input file was given, please check the given filename.";
		CV_Error(CV_StsBadArg, error_message);
	}
	std::string line, path, classlabel;
	while (file.good())
	{
		std::getline(file, path, separator);
		std::getline(file, classlabel, '\n');

		if (!path.empty() && !classlabel.empty()) {
			images.push_back(cv::imread(path, 0));
			labels.push_back(atoi(classlabel.c_str()));
		}
	}
}


     /**
     * @brief   Learn face model
     * @param   fn_csv [std::string] Path to the CSV file with the face database
     * @return  [std::string] The model's URL
     */
std::string FaceRecognizer::learnFace(std::string fn_csv, std::string folder, std::string model_name) // method needs at least two classes to learn a model
	// faces should be visible on the dark homogeneous background
{
	// These vectors hold the images and corresponding labels:
	std::vector<cv::Mat> images;
	std::vector<int> labels;
	
	
	// Read in the data (fails if no valid input filename is given, but you'll get an error message):
	try {
		FaceRecognizer::read_csv(fn_csv, images, labels); 
	}
	catch (cv::Exception& e) {
		std::cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << std::endl;
		// nothing more we can do
		exit(1);
	}

	// Find home directory
	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir; // return path to the home directory
	// Set folder path
	std::string folder_path = homedir + folder;
	// Create user folder, if the directory does not already exists -- for Linux
	struct stat st = {0};
	if (stat(folder_path.c_str(), &st) == -1) {
		mkdir(folder_path.c_str(), 0700);
	}
	
		
	if (images.size() > 1 && images[0].cols>0){
		int im_width = images[0].cols;
		int im_height = images[0].rows;
		// Create a FaceRecognizer and train it on the given images:
		cv::Ptr<cv::FaceRecognizer> model = cv::createFisherFaceRecognizer(); // the Fisherfaces method needs at least two classes to learn a model
		model->train(images, labels);


		// save the model to eigenfaces_recog.yaml
		model->save(folder_path+model_name);
	
		return (folder_path+model_name);
	}
	else{
		std::string error_message = "Face recognizer needs at least two classes to learn a model; Check path to images";
		CV_Error(CV_StsBadArg, error_message);
		return "";
	}
}


     /**
     * @brief   Recognizes a face while using detected faces and face model, which is loaded from a file URL
     * @param   img_ [cv::Mat&] The input image
     * @param   faces_ [std::vector< cv::Rect_<int> >] The vector containing detected faces in the given image
     * @param   model_name_ [const std::string]	The face model's URL
     * @param   predictedConfidenceVec [std::vector< double >&] Vector which will return predicted confidence value
     * @param   face_size_ [cv::Size] Size of face (default size is cv::Size(92,112))
     * @return  [std::vector< int >] The vector of recognized face ID
     */
std::vector< int > FaceRecognizer::recognizeFace(cv::Mat & img_, std::vector< cv::Rect_<int> > faces_, const std::string model_name_, std::vector< double > & predictedConfidenceVec, cv::Size face_size_)
{
	const bool visualizes = false;// visualisation of face recognition
	/// Create a new Eigenfaces Recognizer && load the model
	//
	cv::Ptr<cv::FaceRecognizer> model = cv::createEigenFaceRecognizer();
	
	std::vector< int > prediction_vec;
	predictedConfidenceVec.clear(); // clears the vector

	try{
		model->load(model_name_);// "eigenfaces_recog.yml");
	}
	catch(...)
	{
		std::cerr<<"File don't exists\n";
		return prediction_vec;
	}
	// Convert the current frame to grayscale:
	cv::Mat gray;
	cvtColor(img_, gray, CV_BGR2GRAY);

	


	for (int i = 0; i < faces_.size(); i++) {
		// Process face by face.
		cv::Rect face_i = faces_[i];
		// Crop the face from the image.
		cv::Mat face = gray(face_i);
		// Resizing the face is necessary for Eigenfaces and Fisherfaces.
		// Face resizing
		cv::Mat face_resized;
		cv::resize(face, face_resized, face_size_,//cv::Size(im_width, im_height)
			1.0, 1.0, cv::INTER_CUBIC);
		// Prediction variables
		int predicted_label = -1;
		double predicted_confidence = 0.0;
		// Get the prediction and associated confidence from the model
		model->predict(face_resized, predicted_label, predicted_confidence); // is going to yield - 1 as predicted label, which states this face is unknown.
		//predicted_label = model->predict(face_resized); //is going to yield -1 as predicted label, which states this face is unknown.

		prediction_vec.push_back(predicted_label); // adds the predicted label
		predictedConfidenceVec.push_back(predicted_confidence); // adds the predicted confidence value to the vector

		/// Visualisation
		if (visualizes)
		try{
			// Draw a green rectangle around the detected face on the given image:
			cv::rectangle(img_, face_i, CV_RGB(0, 255, 0), 1);
			if (predicted_confidence>1250){ //distance is to large, so probably face in unknown
				predicted_label = -1;
			}
			// Create the text we will annotate the box with:
			std::string box_text = cv::format("Prediction = %d", predicted_label);
			//std::string box_text = cv::format("Prediction = %d, confidence = %f", predicted_label, predicted_confidence);

			// Calculate the position for annotated text (make sure we don't put illegal values in there):
			int pos_x = std::max(face_i.tl().x - 10, 0);
			int pos_y = std::max(face_i.tl().y - 10, 0);

			// Show recognized face on the image:
			cv::putText(img_, box_text, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
		}
		catch (...)
		{
			std::cout << "Illegal values in calculation of face position \n";
		}
	
	}
	return prediction_vec;
}
