#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/server/simple_action_server.h>
#include <actlib/ForkliftAction.h>
#include <cmath>

class ForkliftAction {
  
  public:
  
    ForkliftAction(std::string name) : as_(nh_, name, false), action_name_(name) {
      ROS_INFO("name: %s", name.c_str());
      as_.registerGoalCallback(boost::bind(&ForkliftAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&ForkliftAction::preempCB, this));
      
      initial_distance_ = -1;
      
      std::string param_topic_pose_,  param_topic_cmd_vel_;
      nh_.getParam("topic_pose", param_topic_pose_);
      nh_.getParam("topic_cmd_vel_", param_topic_cmd_vel_);
      sub_ = nh_.subscribe(param_topic_pose_, 1, &ForkliftAction::getPose, this);
      pub_ = nh_.advertise<geometry_msgs::Twist>(param_topic_cmd_vel_, 1);
      as_.start();
    }
    
    ~ForkliftAction(void) {
      
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
        status_ = 100 * (1 - (distance_ / initial_distance_));
        twist_.linear.x = 1 * distance_;
        twist_.angular.z = 5 * (angle - angle_);
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
    
    void getPose(const geometry_msgs::Pose2D::ConstPtr& msg) {
      angle_ = msg->theta;
      x_ = msg->x;
      y_ = msg->y;

      this->taskToBeDone();
    }
    
  protected:
    
    // mandatory variables
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actlib::ForkliftAction> as_;
    std::string action_name_;
    actlib::ForkliftFeedback feedback_;
    actlib::ForkliftResult result_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    // initial variables
    float initial_distance_, initial_x_, initial_y_;
    // current variables
    float angle_, x_, y_, distance_, status_;
    // goal variables
    geometry_msgs::Pose2D goal_;
    // to publish variables
    geometry_msgs::Twist twist_;
    
};

int main(int argc, char** argv) {
  ros::Time::init();
  ros::init(argc, argv, "forklift_act_srv");
  ros::Rate loop_rate(20);
  
  ForkliftAction forklift(ros::this_node::getName());
  while(ros::ok()) {
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  return 0;
}