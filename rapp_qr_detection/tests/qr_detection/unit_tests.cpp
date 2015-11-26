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

#include <qr_detection/qr_detector.h>
#include <ros/package.h>

/**
 * @class QrDetectionTest
 * @brief Utilizes gtest in order to provide unit tests for qr detection
 */
class QrDetectionTest : public ::testing::Test
{
  protected:
    /**
     * @brief Default constructor
     */
    QrDetectionTest()
    {
    }

    /**
     * @brief Sets up stuff before each unit test call
     */
    virtual void SetUp()
    {
      qr_detector_ = new QrDetector;
    }

    /**
     * @brief Clears up stuff after each unit test call
     */
    virtual void TearDown()
    {
      delete qr_detector_;
    }

    /**< The QrDetector object, used to test its functions */
    QrDetector *qr_detector_;

};

/**
 * @brief Tests QR detection with an image containing face. Should return 0 qrs
 */
TEST_F(QrDetectionTest, lenna_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/Lenna.png");
  std::vector<QrCode> qrs;
  qrs = qr_detector_->findQrs(s);
  EXPECT_EQ(0, qrs.size());
}

/**
 * @brief Tests QR detection with an image containing a qr. Should return 1 qr
 */
TEST_F(QrDetectionTest, qr_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/qr_code_rapp.jpg");
  std::vector<QrCode> qrs;
  qrs = qr_detector_->findQrs(s);
  EXPECT_EQ(1, qrs.size());
}

/**
 * @brief Tests QR detection with a non-existent image. Should return 0 qrs
 */
TEST_F(QrDetectionTest, file_not_exists_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/file_not_exists.jpg");
  std::vector<QrCode> qrs;
  qrs = qr_detector_->findQrs(s);
  EXPECT_EQ(0, qrs.size());
}

/**
 * @brief Tests QR detection with a zero-sized image. Should return 0 qrs
 */
TEST_F(QrDetectionTest, zero_sized_image_test)
{
  cv::Mat tmp_img(0, 0, CV_8UC1);
  std::vector<QrCode> qrs;
  qrs = qr_detector_->detectQrs(tmp_img);
  EXPECT_EQ(0, qrs.size());
}

/**
 * @brief The main function. Initializes the unit tests
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

