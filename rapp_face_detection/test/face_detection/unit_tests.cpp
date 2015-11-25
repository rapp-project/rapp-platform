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

#include <face_detection/face_detector.h>
#include <ros/package.h>

/**
 * @class FaceDetectionTest
 * @brief Handles the face detection unit testing using gtests
 */
class FaceDetectionTest : public ::testing::Test
{
  protected:
    
    /**
     * @brief Default constructor
     */
    FaceDetectionTest()
    {
    }
    /**
     * @brief Sets up the class variables for each unit test call
     */
    virtual void SetUp()
    {
      face_detector_ = new FaceDetector;
    }

    /**
     * @brief This function is called after the termination of each test. Destroys the dynamically alloced variables
     */
    virtual void TearDown()
    {
      delete face_detector_;
    }

    FaceDetector *face_detector_; /**< Pointer of type FaceDetector. Used to check its functions */

};

/**
 * @brief Tests face detection with the Lenna image. Should be successful
 */ 
TEST_F(FaceDetectionTest, lenna_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/Lenna.png");
  std::vector<cv::Rect> faces = face_detector_->findFaces(s);
  EXPECT_EQ(1,faces.size());
}

/**
 * @brief Tests face detection with a qr code. Should return 0 faces
 */
TEST_F(FaceDetectionTest, qr_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/qr_code_rapp.jpg");
  std::vector<cv::Rect> faces = face_detector_->findFaces(s);
  EXPECT_EQ(0,faces.size());
}

/**
 * @brief Tests face detection with a missing file. Should return 0 faces
 */
TEST_F(FaceDetectionTest, file_not_exists_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/not_existent_file.jpg");
  std::vector<cv::Rect> faces = face_detector_->findFaces(s);
  EXPECT_EQ(0,faces.size());
}

/**
 * @brief Tests face detection with an empty image. Should return 0 faces
 */
TEST_F(FaceDetectionTest, zero_sized_image_test)
{
  cv::Mat tmp_img(0, 0, CV_8UC1);
  std::vector<cv::Rect> faces = face_detector_->detectFaces(tmp_img);
  EXPECT_EQ(0,faces.size());
}

/**
 * @brief The main function. Initialized the unit tests
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

