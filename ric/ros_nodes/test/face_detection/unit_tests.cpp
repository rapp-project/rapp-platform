#include <gtest/gtest.h>

#include <face_detection/face_detection.h>

class FaceDetectionTest : public ::testing::Test
{
  protected:
    FaceDetectionTest()
    {
    }
    virtual void SetUp()
    {
      face_detection_ = new FaceDetection;
    }
    virtual void TearDown()
    {
      delete face_detection_;
    }

    FaceDetection *face_detection_;

};

TEST_F(FaceDetectionTest, test1){
  std::string s("/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png");
  std::vector<cv::Rect> faces = face_detection_->findFaces(s);
  EXPECT_EQ(1,faces.size());
}


