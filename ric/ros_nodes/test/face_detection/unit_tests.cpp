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
    }
    virtual void TearDown()
    {
    }

    FaceDetection *face_detection_;

};

TEST_F(FaceDetectionTest, test1){
  EXPECT_EQ(1,1);
}
TEST_F(FaceDetectionTest, test2){
  EXPECT_EQ(1,5);
}

