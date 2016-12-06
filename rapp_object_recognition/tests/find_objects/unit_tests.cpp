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

#include "rapp_object_recognition/find_objects.hpp"
#include <ros/package.h>

/**
 * \class ObjectDetectionTest
 * \brief Handles the object detection unit testing using gtests
 */
class ObjectDetectionTest : public ::testing::Test
{
  protected:
    
    /**
     * \brief Default constructor
     */
    ObjectDetectionTest()
    {
    }
    /**
     * \brief Sets up the class variables for each unit test call
     */
    virtual void SetUp()
    {
      find_objects_ = new FindObjects();
    }

    /**
     * \brief This function is called after the termination of each test. Destroys the dynamically alloced variables
     */
    virtual void TearDown()
    {
      delete find_objects_;
    }

    FindObjects *find_objects_; /**< Pointer of type FindObjects. Used to check its functions */

};

/**
 * \brief Tests object model learning and loading.
 */ 
TEST_F(ObjectDetectionTest, test_learn)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  bool cl = find_objects_->clearModels("test");
  EXPECT_TRUE(cl);
  
  std::string file_path = path + std::string("/test_data/object_recognition_samples/book_1/cat.jpg");
  int ln = find_objects_->learnObject("test", file_path, "cat");
  EXPECT_EQ(ln, 0);
  
  ln = find_objects_->learnObject("test", file_path+"foo", "catfoo");
  EXPECT_EQ(ln, -2);
  
  std::vector<std::string> f_names;
  std::vector<double> f_scores;
  std::vector<geometry_msgs::Point> f_centers;
  int fd = find_objects_->findObjects("test", file_path, 1, f_names, f_centers, f_scores);
  EXPECT_EQ(fd, -1);
  
  int ld_res;
  std::map<std::string, bool> ld = find_objects_->loadModels("test", {"cat", "catfoo"}, ld_res);
  EXPECT_EQ(ld.size(), 2);
  EXPECT_TRUE(ld["cat"]);
  EXPECT_FALSE(ld["catfoo"]);
}

/**
 * \brief Tests object detection with no models loaded. 
 * 
 * Should return status=-1
 */ 
TEST_F(ObjectDetectionTest, test_clear)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  bool cl = find_objects_->clearModels("test");
  EXPECT_TRUE(cl);
  
  std::string file_path = path + std::string("/test_data/object_recognition_samples/book_1/cat.jpg");
  std::vector<std::string> f_names;
  std::vector<double> f_scores;
  std::vector<geometry_msgs::Point> f_centers;
  int fd = find_objects_->findObjects("test", file_path, 1, f_names, f_centers, f_scores);
  EXPECT_EQ(fd, -1);
}

/**
 * \brief Tests object detection with some models.
 */ 
TEST_F(ObjectDetectionTest, test_detect_existing)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  bool cl = find_objects_->clearModels("test");
  EXPECT_TRUE(cl);
  
  std::string file_path = path + std::string("/test_data/object_recognition_samples/book_1/cat.jpg");
  int ln = find_objects_->learnObject("test", file_path, "cat");
  EXPECT_EQ(ln, 0);
  
  int ld_res;
  std::map<std::string, bool> ld = find_objects_->loadModels("test", {"cat"}, ld_res);
  EXPECT_EQ(ld.size(), 1);
  EXPECT_TRUE(ld["cat"]);
  
  std::vector<std::string> f_names;
  std::vector<double> f_scores;
  std::vector<geometry_msgs::Point> f_centers;
  int fd = find_objects_->findObjects("test", file_path, 1, f_names, f_centers, f_scores);
  EXPECT_EQ(fd, 0);
  EXPECT_EQ(f_names.size(), 1);
  EXPECT_EQ(f_names[0], "cat");
}

/**
 * \brief Tests object detection with wrong models.
 */ 
TEST_F(ObjectDetectionTest, test_detect_not_existing)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  bool cl = find_objects_->clearModels("test");
  EXPECT_TRUE(cl);
  
  std::string file_path = path + std::string("/test_data/object_recognition_samples/book_1/cat.jpg");
  std::string file_path_dog = path + std::string("/test_data/object_recognition_samples/book_1/dog.jpg");
  int ln = find_objects_->learnObject("test", file_path, "cat");
  EXPECT_EQ(ln, 0);
  
  int ld_res;
  std::map<std::string, bool> ld = find_objects_->loadModels("test", {"cat"}, ld_res);
  EXPECT_EQ(ld.size(), 1);
  EXPECT_TRUE(ld["cat"]);
  
  std::vector<std::string> f_names;
  std::vector<double> f_scores;
  std::vector<geometry_msgs::Point> f_centers;
  int fd = find_objects_->findObjects("test", file_path_dog, 1, f_names, f_centers, f_scores);
  EXPECT_EQ(fd, 0);
  EXPECT_EQ(f_names.size(), 0);
}

/**
 * \brief Tests object detection with multiple objects on scene.
 */ 
TEST_F(ObjectDetectionTest, test_detect_multiple)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  bool cl = find_objects_->clearModels("test");
  EXPECT_TRUE(cl);
  
  std::string file_path_cat = path + std::string("/test_data/object_recognition_samples/book_1/cat.jpg");
  std::string file_path_dog = path + std::string("/test_data/object_recognition_samples/book_1/dog.jpg");
  int ln = find_objects_->learnObject("test", file_path_cat, "cat");
  EXPECT_EQ(ln, 0);
  
  ln = find_objects_->learnObject("test", file_path_dog, "dog");
  EXPECT_EQ(ln, 0);
  
  int ld_res;
  std::map<std::string, bool> ld = find_objects_->loadModels("test", {"cat", "dog"}, ld_res);
  EXPECT_EQ(ld.size(), 2);
  EXPECT_TRUE(ld["cat"]);
  EXPECT_TRUE(ld["dog"]);
  
  std::vector<std::string> f_names;
  std::vector<double> f_scores;
  std::vector<geometry_msgs::Point> f_centers;
  
  std::string file_path_scene = path + std::string("/test_data/object_recognition_samples/cat_2_dog_1.jpg");
  int fd = find_objects_->findObjects("test", file_path_scene, 1, f_names, f_centers, f_scores);
  EXPECT_EQ(fd, 0);
  EXPECT_EQ(f_names.size(), 1);
  
  fd = find_objects_->findObjects("test", file_path_scene, 10, f_names, f_centers, f_scores);
  EXPECT_EQ(fd, 0);
  EXPECT_EQ(f_names.size(), 3);
  EXPECT_EQ(std::count (f_names.begin(), f_names.end(), "cat"), 2);
  EXPECT_EQ(std::count (f_names.begin(), f_names.end(), "dog"), 1);
}

/**
 * \brief Tests object detection with pictures from NAO camera.
 */ 
TEST_F(ObjectDetectionTest, test_detect_nao)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  
  // clear models cache
  bool cl = find_objects_->clearModels("test");
  EXPECT_TRUE(cl);
  
  // learn new objects
  std::vector<std::string> models = {"dilmah", "lipton", "mieta"};
  for (auto const & m: models) {
    int ln = find_objects_->learnObject("test", path + "/test_data/object_recognition_samples/nao_teas/" + m + ".png", m);
    EXPECT_EQ(ln, 0);
  }
  
  int ld_res;
  auto ld = find_objects_->loadModels("test", models, ld_res);
  EXPECT_EQ(ld.size(), 3);
  
  
  std::vector<std::string> f_names;
  std::vector<double> f_scores;
  std::vector<geometry_msgs::Point> f_centers;
  
  std::vector<std::string> scenes = {"0", "1", "2", "3", "4", "5", "6", "7"};
  for (auto const & scene: scenes) {
    std::string file_path_scene = path + std::string("/test_data/object_recognition_samples/nao_teas/" + scene + ".png");
    int fd = find_objects_->findObjects("test", file_path_scene, 10, f_names, f_centers, f_scores);
  }
}

/**
 * \brief Tests loading models by keyword
 * 
 * 
 */ 
TEST_F(ObjectDetectionTest, test_keywords)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  bool cl = find_objects_->clearModels("test");
  EXPECT_TRUE(cl);

  int status;
  find_objects_->loadKeywords("test", {"a", "b", "c"}, status);

}


/**
 * \brief The main function. Initializes the unit tests
 */
int main(int argc, char **argv)
{
  //testing::GTEST_FLAG(filter) = "*multiple*";//":-:*Counter*";
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

