#include <gtest/gtest.h>

#include <qr_detection/qr_detector.h>

class QrDetectionTest : public ::testing::Test
{
  protected:
    QrDetectionTest()
    {
    }
    virtual void SetUp()
    {
      qr_detector_ = new QrDetector;
    }
    virtual void TearDown()
    {
      delete qr_detector_;
    }

    QrDetector *qr_detector_;

};

TEST_F(QrDetectionTest, test1){
  std::string s("/home/etsardou/rapp_platform_catkin_ws/src/rapp-platform/ric/test_auxiliary_files/qr_code_rapp.jpg");
  std::vector<cv::Point> points;
  std::vector<std::string> messages;
  qr_detector_->findQrs(s, points, messages);
  EXPECT_EQ(1,messages.size());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

