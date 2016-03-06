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

#include <hazard_detection/light_check.hpp>
#include <hazard_detection/door_check.hpp>
#include <ros/package.h>

/**
 * @class LightCheckTest
 * @brief Handles the light cheking unit testing using gtests
 */
class LightCheckTest : public ::testing::Test
{
  protected:
    
    /**
     * @brief Default constructor
     */
    LightCheckTest()
    {
    }
    /**
     * @brief Sets up the class variables for each unit test call
     */
    virtual void SetUp()
    {
      light_check_ = new LightCheck;
    }

    /**
     * @brief This function is called after the termination of each test. Destroys the dynamically alloced variables
     */
    virtual void TearDown()
    {
      delete light_check_;
    }

    LightCheck *light_check_; /**< Pointer of type LightCheck. Used to check its functions */

};

/**
 * @brief Tests light detection with the lamp turned on. Should be successful
 */ 
TEST_F(LightCheckTest, test_on)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/hazard_detection_samples/lamp_on.jpg");
  int light_level = light_check_->process(s);
  EXPECT_GT(light_level, 50);
}

/**
 * @brief Tests light detection with the lamp turned off. Should be successful
 */ 
TEST_F(LightCheckTest, test_off)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/hazard_detection_samples/lamp_off.jpg");
  int light_level = light_check_->process(s);
  EXPECT_LT(light_level, 50);
  EXPECT_GE(light_level, 0);
}

/**
 * @brief Tests light detection on the same image with different scales
 */ 
TEST_F(LightCheckTest, test_size)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s1 = path + std::string("/test_data/hazard_detection_samples/lamp_on.jpg");
  std::string s2 = path + std::string("/test_data/hazard_detection_samples/lamp_on_small.jpg");
  int light_level_1 = light_check_->process(s1);
  int light_level_2 = light_check_->process(s2);
  EXPECT_EQ(light_level_1, light_level_2);
}

/**
 * @brief Tests light detection with a missing file. Should return 0
 */
TEST_F(LightCheckTest, file_not_exists_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/not_existent_file.jpg");
  int light_level = light_check_->process(s);
  EXPECT_EQ(light_level, -1);
}




/**
 * @class DoorCheckTest
 * @brief Handles the door angle detection unit testing using gtests
 */
class DoorCheckTest : public ::testing::Test
{
  protected:
    
    /**
     * @brief Default constructor
     */
    DoorCheckTest()
    {
    }
    /**
     * @brief Sets up the class variables for each unit test call
     */
    virtual void SetUp()
    {
      door_check_ = new DoorCheck;
    }

    /**
     * @brief This function is called after the termination of each test. Destroys the dynamically alloced variables
     */
    virtual void TearDown()
    {
      delete door_check_;
    }

    DoorCheck *door_check_; /**< Pointer of type DoorCheck. Used to check its functions */

};

/**
 * @brief Tests light detection with a missing file. Should return 0
 */
TEST_F(DoorCheckTest, file_not_exists_test)
{
  std::string path = ros::package::getPath("rapp_testing_tools");
  std::string s = path + std::string("/test_data/not_existent_file.jpg");
  int door_angle = door_check_->process(s);
  EXPECT_EQ(-1, door_angle);
}



/**
 * @brief The main function. Initialized the unit tests
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

