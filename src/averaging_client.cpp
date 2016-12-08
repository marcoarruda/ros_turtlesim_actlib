#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actlib/AveragingAction.h>
#include <boost/thread.hpp>

void spinThread() {
  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_averaging");
  
  actionlib::SimpleActionClient<actlib::AveragingAction> ac("averaging");
  boost::thread spin_thread(&spinThread);
  
  ROS_INFO("waiting for action server to start");
  ac.waitForServer();
  
  ROS_INFO("Action server started, sending goal");
  actlib::AveragingGoal goal;
  goal.samples = 100;
  ac.sendGoal(goal);
  
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  
  if(finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("Action did not finished before the time out");
  }
  
  ros::shutdown();
  spin_thread.join();
  
  return 0;
}