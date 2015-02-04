#include <gtest/gtest.h>

#include <face_detection/face_detector.h>

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

TEST_F(FaceDetectionTest, test1){
  std::string s("/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png");
  std::vector<cv::Rect> faces = face_detector_->findFaces(s);
  EXPECT_EQ(1,faces.size());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

