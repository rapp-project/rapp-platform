#include <gtest/gtest.h>
#include <ros/ros.h>
#include <navfn/MakeNavPlanResponse.h>
#include <path_planning/path_planner.h>
#include <path_planning/path_planning.h>
#include <ros/package.h>

class PathPlanningTest : public ::testing::Test
{
  protected:
    PathPlanningTest()
    {

    }
    virtual void SetUp()
    {
      path_planner_ = new PathPlanner;

    }
    virtual void TearDown()
    {
      delete path_planner_;

    }

    PathPlanner *path_planner_;

};

TEST_F(PathPlanningTest, setSequenceNR_test)
{

  ros::NodeHandle nh;
  nh.setParam("/rapp/rapp_path_planning/last_seq", 1);
  std::string new_seq = path_planner_->setSequenceNR(nh,5);
  EXPECT_EQ("2",new_seq);
}

TEST_F(PathPlanningTest, configureSequence_test)
{
  ros::NodeHandle nh;
  std::string map_path = "/home/rapp/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_map_server/maps/empty.yaml";
  bool status = path_planner_->configureSequence("2",map_path,"NAO","dijkstra",nh);
  EXPECT_EQ(true,status);
}

TEST_F(PathPlanningTest, startSequence_test)
{
  navfn::MakeNavPlanResponse planned_path; 

  ros::NodeHandle nh;
  geometry_msgs::PoseStamped request_start, request_goal;
  request_start.pose.position.x = 1;
  request_start.pose.position.y = 1;

  request_goal.pose.position.x = 3;
  request_goal.pose.position.y = 3;

  std::string map_path = "/home/rapp/rapp_platform/rapp-platform-catkin-ws/src/rapp-platform/rapp_map_server/maps/empty.yaml";
  bool status = path_planner_->configureSequence("2",map_path,"NAO","dijkstra",nh);

  planned_path = path_planner_->startSequence("2", request_start,request_goal, nh);
  EXPECT_EQ(1,planned_path.plan_found);
}


int main(int argc, char **argv)
{
      pid_t roscore_pID = fork();
         if (roscore_pID == 0)                
         {
 
            execl("/opt/ros/indigo/bin/roscore", "/opt/ros/indigo/bin/roscore", (char *)0);
          }
          else if (roscore_pID < 0)           
          {
              std::cout << "Failed to start roscore "<< std::endl;
              exit(1);
              
          }
          else{
		ros::init(argc, argv, "unit_test_path_planning");
		    PathPlanning *path_planning_;
		      path_planning_ = new PathPlanning;
		  testing::InitGoogleTest(&argc, argv);
		      delete path_planning_;
		  return RUN_ALL_TESTS();
          }



}

