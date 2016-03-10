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

#include <human_detection/human_detector.h>
#include <ros/package.h>

/**
 * @class HumanDetectionTest
 * @brief Handles the human detection unit testing using gtests
 */
class HumanDetectionTest : public ::testing::Test
{
  protected:
    
    /**
     * @brief Default constructor
     */
    HumanDetectionTest()
    {
    }
    /**
     * @brief Sets up the class variables for each unit test call
     */
    virtual void SetUp()
    {
      human_detector_ = new HumanDetector;
    }

    /**
     * @brief This function is called after the termination of each test. Destroys the dynamically alloced variables
     */
    virtual void TearDown()
    {
      delete human_detector_;
    }

    HumanDetector *human_detector_; /**< Pointer of type HumanDetector. Used to check its functions */

};

/**
 * @brief Tests human detection with the Lenna image. Should be successful
 */ 
TEST_F(HumanDetectionTest, human_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/human_detection_samples/NAO_picture_3.png");
  std::vector<cv::Rect> humans = human_detector_->findHuman2D(s);
  EXPECT_EQ(1,humans.size());
}

/**
 * @brief Tests human detection with a qr code. Should return 0 humans
 */
TEST_F(HumanDetectionTest, qr_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/qr_code_rapp.jpg");
  std::vector<cv::Rect> humans = human_detector_->findHuman2D(s);
  EXPECT_EQ(0,humans.size());
}

/**
 * @brief Tests human detection with a missing file. Should return 0 humans
 */
TEST_F(HumanDetectionTest, file_not_exists_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/not_existent_file.jpg");
  std::vector<cv::Rect> humans = human_detector_->findHuman2D(s);
  EXPECT_EQ(0,humans.size());
}

/**
 * @brief Tests human detection with an empty image. Should return 0 humans
 */
TEST_F(HumanDetectionTest, zero_sized_image_test)
{
  cv::Mat tmp_img(0, 0, CV_8UC1);
  std::vector<cv::Rect> humans = human_detector_->detectHuman2D(tmp_img);
  EXPECT_EQ(0,humans.size());
}

/**
 * @brief The main function. Initialized the unit tests
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

