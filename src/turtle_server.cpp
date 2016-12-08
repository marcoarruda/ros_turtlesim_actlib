#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <actlib/TurtleAction.h>
#include <cmath>

class TurtleAction {
  
  public:
  
    TurtleAction(std::string name) : as_(nh_, name, false), action_name_(name) {
      ROS_INFO("name: %s", name.c_str());
      as_.registerGoalCallback(boost::bind(&TurtleAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&TurtleAction::preempCB, this));
      
      initial_distance_ = -1;
      
      sub_ = nh_.subscribe("/turtle1/pose", 1, &TurtleAction::getPose, this);
      pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
      as_.start();
    }
    
    ~TurtleAction(void) {
      
    }

    void taskToBeDone() {
      if(!as_.isActive()) return;
      
      distance_ = sqrt(pow(x_ - goal_.x, 2) + pow(y_ - goal_.y, 2));
      
      // publish result
      if(distance_ < 0.05) {
        result_.x = x_;
        result_.y = y_;
        result_.distance = distance_;
        result_.status = status_;

        twist_.linear.x = 0;
        twist_.angular.z = 0;
        pub_.publish(twist_);

        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
        return;
      } else {
        // math for velocity command publishing
        float angle = atan2(goal_.y - y_, goal_.x - x_);
        float absv = std::abs(angle - angle_);
        //ROS_INFO("angle error: %.2f", angle);
        status_ = 100 * (1 - (distance_ / initial_distance_));
        if(absv > 0.1) {
          twist_.linear.x = 0;
          twist_.angular.z = 2 * (angle - angle_);
        } else {
          twist_.linear.x = 2 * distance_;
          twist_.angular.z = 0;
        }
      }

      pub_.publish(twist_);
      
      // publish feedback
      feedback_.x = x_;
      feedback_.y = y_;
      feedback_.distance = distance_;
      feedback_.status = status_;
      as_.publishFeedback(feedback_);
    }
    
    void goalCB() {
      goal_ = as_.acceptNewGoal()->pose;
      distance_ = sqrt(pow(x_ - goal_.x, 2) + pow(y_ - goal_.y, 2));
      if(initial_distance_ == -1) {
        initial_x_ = x_;
        initial_y_ = y_;
        initial_distance_ = distance_;
      }
      ROS_INFO("New goal accepted!");
    }
    
    void preempCB() {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }
    
    void getPose(const turtlesim::Pose::ConstPtr& msg) {
      angle_ = msg->theta;
      x_ = msg->x;
      y_ = msg->y;

      this->taskToBeDone();
    }
    
  protected:
    
    // mandatory variables
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actlib::TurtleAction> as_;
    std::string action_name_;
    actlib::TurtleFeedback feedback_;
    actlib::TurtleResult result_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    // initial variables
    float initial_distance_, initial_x_, initial_y_;
    // current variables
    float angle_, x_, y_, distance_, status_;
    // goal variables
    turtlesim::Pose goal_;
    // to publish variables
    geometry_msgs::Twist twist_;
    
};

int main(int argc, char** argv) {
  ros::Time::init();
  ros::init(argc, argv, "turtle");
  ros::Rate loop_rate(20);
  
  TurtleAction turtle(ros::this_node::getName());
  while(ros::ok()) {
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  return 0;
}