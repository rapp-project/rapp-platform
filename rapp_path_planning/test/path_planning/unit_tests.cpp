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

  TEST_F(PathPlanningTest, startSequenceNodes_test)
  {
    ros::NodeHandle nh;
    nh.setParam("/rapp_path_planning_threads", 1);

    PathPlanning *path_planning_;
    path_planning_ = new PathPlanning;

    uint32_t serv_port;
    std::string serv_node_name, serv_name;
    ros::ServiceManager srv_manager;
    bool serv_active = srv_manager.lookupService("/global_planner1/make_plan",serv_node_name,serv_port);
    int i=0;
    while (!serv_active && i <= 20){
    ROS_ERROR_STREAM("Waiting for:"<<" /global_planner1/make_plan" << " service ");
    serv_active = srv_manager.lookupService("/global_planner1/make_plan",serv_node_name,serv_port);
    ros::Duration(1).sleep();
    i+=1;
    }
    bool status;
  if (i<=20)
   status = true;
  else
    status = false;
  delete path_planning_;

    // std::string map_path = ros::package::getPath("rapp_map_server")+"/maps/empty.yaml";
    // bool status = path_planner_->configureSequence("2",map_path,"NAO","dijkstra",nh);
     EXPECT_EQ(true,status);
   }

// TEST_F(PathPlanningTest, startSequence_test)
// {
//   navfn::MakeNavPlanResponse planned_path; 

//   ros::NodeHandle nh;
//   geometry_msgs::PoseStamped request_start, request_goal;
//   request_start.pose.position.x = 1;
//   request_start.pose.position.y = 1;

//   request_goal.pose.position.x = 3;
//   request_goal.pose.position.y = 3;

//   std::string map_path = ros::package::getPath("rapp_map_server")+"/maps/empty.yaml";
//   bool status = path_planner_->configureSequence("2",map_path,"NAO","dijkstra",nh);

//   planned_path = path_planner_->startSequence("2", request_start,request_goal, nh);
//   EXPECT_EQ(1,planned_path.plan_found);
// }


int main(int argc, char **argv)
{
//  uint32_t serv_port;
//   std::string serv_node_name, serv_name;
//   ros::ServiceManager srv_manager; 
//   int status;
//  bool serv_active = srv_manager.lookupService("/global_planner1/make_plan",serv_node_name,serv_port);
//  if (!serv_active){
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
               ros::NodeHandle nh;

 uint32_t serv_port;
  std::string serv_node_name, serv_name;
  ros::ServiceManager srv_manager;
  bool serv_active = srv_manager.lookupService("/rosout/get_loggers",serv_node_name,serv_port);
  int i=0;
  while (!serv_active && i <= 20){
      ROS_WARN_STREAM("Waiting for:"<<" rosmaster" );
      serv_active = srv_manager.lookupService("/rosout/get_loggers",serv_node_name,serv_port);
      ros::Duration(1).sleep();
      i+=1;
    }

            testing::InitGoogleTest(&argc, argv);
         int status =  RUN_ALL_TESTS();
         kill(roscore_pID, SIGTERM);
         sleep(2);
         kill(roscore_pID, SIGKILL);
         return status;
}
//           int argc = 0;
//           char ** argv = NULL;
//           ros::init(argc, argv, "unit_test_path_planning");
//           PathPlanning *path_planning_;
//           path_planning_ = new PathPlanning;

//           testing::InitGoogleTest(&argc, argv);

//           int test_status =  RUN_ALL_TESTS();
// kill(roscore_pID, SIGTERM);
//         sleep(2);
//         kill(roscore_pID, SIGKILL);
//                             delete path_planning_;

//         return test_status;

// }}else{
//           int argc = 0;
//           char ** argv = NULL;
//           PathPlanning *path_planning_;
//           path_planning_ = new PathPlanning;

//           testing::InitGoogleTest(&argc, argv);

//           int test_status =  RUN_ALL_TESTS();
//                               delete path_planning_;

//           return test_status;



// }
// ros::init(argc, argv, "unit_test_path_planning");


        }
