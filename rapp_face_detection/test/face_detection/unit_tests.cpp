#include <gtest/gtest.h>

#include <face_detection/face_detector.h>
#include <ros/package.h>

class FaceDetectionTest : public ::testing::Test
{
  protected:
    FaceDetectionTest()
    {
    }
    virtual void SetUp()
    {
      face_detector_ = new FaceDetector;
    }
    virtual void TearDown()
    {
      delete face_detector_;
    }

    FaceDetector *face_detector_;

};

TEST_F(FaceDetectionTest, lenna_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/testing_tools/test_data/Lenna.png");
  std::vector<cv::Rect> faces = face_detector_->findFaces(s);
  EXPECT_EQ(1,faces.size());
}

TEST_F(FaceDetectionTest, qr_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/testing_tools/test_data/qr_code_rapp.jpg");
  std::vector<cv::Rect> faces = face_detector_->findFaces(s);
  EXPECT_EQ(0,faces.size());
}

TEST_F(FaceDetectionTest, file_not_exists_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/testing_tools/test_data/not_existent_file.jpg");
  std::vector<cv::Rect> faces = face_detector_->findFaces(s);
  EXPECT_EQ(0,faces.size());
}

TEST_F(FaceDetectionTest, zero_sized_image_test)
{
  cv::Mat tmp_img(0, 0, CV_8UC1);
  std::vector<cv::Rect> faces = face_detector_->detectFaces(tmp_img);
  EXPECT_EQ(0,faces.size());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

