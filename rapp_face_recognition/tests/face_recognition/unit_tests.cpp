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

#include <gtest/gtest.h>

#include <face_recognition/face_recognizer.h>
#include <ros/package.h>
#include <vector>

/**
 * @class FaceRecognitionTest
 * @brief Handles the face recognition unit testing using gtests
 */
class FaceRecognitionTest : public ::testing::Test
{
  protected:
    
    /**
     * @brief Default constructor
     */
    FaceRecognitionTest()
    {
    }
    /**
     * @brief Sets up the class variables for each unit test call
     */
    virtual void SetUp()
    {
      face_recognizer_ = new FaceRecognizer;
    }

    /**
     * @brief This function is called after the termination of each test. Destroys the dynamically alloced variables
     */
    virtual void TearDown()
    {
      delete face_recognizer_;
    }

    FaceRecognizer *face_recognizer_; /**< Pointer of type FaceRecognizer. Used to check its functions */

};

/**
 * @brief Tests face recognition with the given image.
 */ 
TEST_F(FaceRecognitionTest, face_recognition_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/face_samples/face_recog_1.png");
  // Load the image
  cv::Mat img = face_recognizer_->loadImage(s);
  // Detect faces
  std::vector< cv::Rect_<int> > faces;
  cv::CascadeClassifier haar_cascade;
  cv::Mat gray;
  //haar_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2_new.xml");
  haar_cascade.load(path + "/test_data/face_samples/face_recognition_model/haarcascade_frontalface_alt2_new.xml");
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  haar_cascade.detectMultiScale(gray, faces);
  const std::string model_name = path + std::string("/test_data/face_samples/face_recognition_model/eigenfaces_recog2.yml");
  std::vector< double > predictedConfidenceVec;
  std::vector< int > faceIDs = face_recognizer_->recognizeFace(img, faces, model_name, predictedConfidenceVec);
  //std::cout<<img.size() << faces.size() << "; " << faceIDs.size() <<std::endl;
  EXPECT_EQ(11,faceIDs[0]);
}

/**
 * @brief Tests face recognition with the given image. Should return unknown face ID (-1)
 */ 
TEST_F(FaceRecognitionTest, unknown_face_recognition_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/face_samples/etsardou_near.jpg");
  // Load the image
  cv::Mat img = face_recognizer_->loadImage(s);
  // Detect faces
  std::vector< cv::Rect_<int> > faces;
  cv::CascadeClassifier haar_cascade;
  cv::Mat gray;
  haar_cascade.load(path + "/test_data/face_samples/face_recognition_model/haarcascade_frontalface_alt2_new.xml");
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  haar_cascade.detectMultiScale(gray, faces);
  const std::string model_name = path + std::string("/test_data/face_samples/face_recognition_model/eigenfaces_recog2.yml");
  std::vector< double > predictedConfidenceVec;
  // Run face recognition
  std::vector< int > faceIDs = face_recognizer_->recognizeFace(img, faces, model_name, predictedConfidenceVec);
  if(predictedConfidenceVec[0]>1250)faceIDs[0]=-1;
  EXPECT_EQ(-1,faceIDs[0]);
}

/**
 * @brief Tests face recognition with a missing model file. Should return 0
 */
TEST_F(FaceRecognitionTest, file_not_exists_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/face_samples/multi_faces_frames/two_faces.jpg");
  // Load the image
  cv::Mat img = face_recognizer_->loadImage(s);
  // Detect faces
  std::vector< cv::Rect_<int> > faces;
  cv::CascadeClassifier haar_cascade;
  cv::Mat gray;
  haar_cascade.load(path + "/test_data/face_samples/face_recognition_model/haarcascade_frontalface_alt2_new.xml");
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  haar_cascade.detectMultiScale(gray, faces);
  // Recognize faces
  const std::string model_name = path + std::string("/test_data/face_samples/face_recognition_model/not_existent_file.yml");
  std::vector< double > predictedConfidenceVec;
  std::vector< int > faceIDs = face_recognizer_->recognizeFace(img, faces, model_name, predictedConfidenceVec);
  EXPECT_EQ(0,faceIDs.size());
}

/**
 * @brief Tests learn face model
 */
TEST_F(FaceRecognitionTest, learn_face_model_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/face_samples/multi_faces_frames/two_faces.jpg");
  // Load the image
  cv::Mat img = face_recognizer_->loadImage(s);
  // Detect faces
  std::vector< cv::Rect_<int> > faces;
  cv::CascadeClassifier haar_cascade;
  cv::Mat gray;
  haar_cascade.load(path + "/test_data/face_samples/face_recognition_model/haarcascade_frontalface_alt2_new.xml");
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  haar_cascade.detectMultiScale(gray, faces);
  // Learn faces
  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir; // return path to the home directory
  std::string path_model = "/rapp_platform_files/";
  std::string user = "Jan";
  std::string model_name = face_recognizer_->learnFace(path + std::string("/test_data/face_samples/face_recognition_model/faces-s.csv"), path_model+user+std::string("/"), std::string("face_model.yml")); // run learnFace
  // Recognize faces
  std::vector< double > predictedConfidenceVec;
  std::vector< int > faceIDs = face_recognizer_->recognizeFace(img, faces, model_name, predictedConfidenceVec);
  EXPECT_EQ(model_name, homedir + path_model + user + std::string("/") + std::string("face_model.yml") );
}
/**
 * @brief The main function. Initialized the unit tests
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

