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

******************************************************************************/

#include <face_recognition/face_recognition.h>


/** 
 * @brief Default constructor. Performs initializations.
 */
FaceRecognition::FaceRecognition(void)
{
  // Fetching the service topic URI parameter
  if(!nh_.getParam("/rapp_face_recognition_recognize_faces_topic", faceRecognitionTopic_))
  {
    ROS_ERROR("Face recognition topic param does not exist");
  }
  // Creating the service server concerning the face recognition functionality
  faceRecognitionService_ = nh_.advertiseService(faceRecognitionTopic_,
    &FaceRecognition::faceRecognitionCallback, this);
}

/**
 * @brief Serves the face recognition ROS service callback
 * @param req [rapp_platform_ros_communications::FaceRecognitionRosSrv::Request&] The ROS service request
 * @param res [rapp_platform_ros_communications::FaceRecognitionRosSrv::Response&] The ROS service response
 * @return bool - The success status of the call
 */
bool FaceRecognition::faceRecognitionCallback(
  rapp_platform_ros_communications::FaceRecognitionRosSrv::Request& req,
  rapp_platform_ros_communications::FaceRecognitionRosSrv::Response& res)
{

	std::string folder_path = "/rapp_platform_files/";
	std::string folder = folder_path + req.user + std::string("/");
	std::string model_name = req.model_name;

	if (req.learn == true){
		model_name = face_recognizer_.learnFace(req.fn_csv, folder, model_name); // run learnFace
		
		//responses
		res.model_name = model_name;
	}//: if
	else{
		res.model_name = req.model_name;
	}

	std::vector< int > recognizedIDs;
	std::vector< double > predictedConfidenceVec;
	if (req.recognize == true){
		cv::Mat img = face_recognizer_.loadImage(req.imageFilename);
		std::vector< cv::Rect_<int> > faces;
		
		for(unsigned int i = 0 ; i < req.faces_up_left.size() ; i++)
		{
			cv::Rect_<int> faceRect;
			unsigned int temp[4];
			temp[0] = req.faces_up_left[i].point.x;
			temp[1] = req.faces_up_left[i].point.y;
			temp[2] = req.faces_down_right[i].point.x - temp[0];
			temp[3] = req.faces_down_right[i].point.y - temp[1];
			faceRect.x = temp[0];
			faceRect.y = temp[1];
			faceRect.width = temp[2];
			faceRect.height = temp[3];
			faces.push_back(faceRect);

		}//: for

		recognizedIDs = face_recognizer_.recognizeFace(img, faces, folder, model_name, predictedConfidenceVec); // run recognizeFace

	}//: if
	//responses
	res.recognizedIDs.swap(recognizedIDs);
	res.predictedConfidenceVec.swap(predictedConfidenceVec);

	return true;
}
