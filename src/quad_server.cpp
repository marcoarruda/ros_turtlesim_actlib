#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <actlib/QuadAction.h>
#include <cmath>

class QuadAction {
  
  public:
  
    QuadAction(std::string name) : as_(nh_, name, false), action_name_(name) {
      ROS_INFO("name: %s", name.c_str());
      as_.registerGoalCallback(boost::bind(&QuadAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&QuadAction::preempCB, this));
      
      initial_distance_ = -1;
      
      sub_ = nh_.subscribe("/vrep/pose", 1, &QuadAction::getPose, this);
      pub1_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/vrep/cmd_attitude", 1);
      pub2_ = nh_.advertise<std_msgs::Float32>("/vrep/cmd_thrust", 1);
      as_.start();
    }
    
    ~QuadAction(void) {
      
    }

    void calcDistance() {
        distance_ = sqrt(pow(z_ - goal_.z, 2) + pow(x_ - goal_.x, 2) + pow(y_ - goal_.y, 2));
    }

    void taskToBeDone() {
      if(!as_.isActive()) return;

      calcDistance();

      // publish result
      if(distance_ < 0.05) {
        result_.pose.x = x_;
        result_.pose.y = y_;
        result_.pose.z = z_;
        result_.distance = distance_;
        result_.status = status_;

        attitude_.vector.x = 0;
        attitude_.vector.y = 0;
        attitude_.vector.z = 0;
        pub1_.publish(attitude_);
        thrust_.data = 0;
        pub2_.publish(thrust_);

        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
        return;
      }

      // math for velocity command publishing
      double kp[3] = {0.1, 0.1, 0.1};
      double ffz = 1.2 * 9.81;
      attitude_.vector.x = -kp[0]*(goal_.x - x_);
      attitude_.vector.y = kp[1]*(goal_.y - y_);
      attitude_.vector.z = 0;
      pub1_.publish(attitude_);
      thrust_.data = ffz; + kp[2]*(goal_.z - z_);
      pub2_.publish(thrust_);
      status_ = 100 * (1 - (distance_ / initial_distance_));
      
      // publish feedback
      feedback_.pose.x = x_;
      feedback_.pose.y = y_;
      feedback_.pose.z = z_;
      feedback_.distance = distance_;
      feedback_.status = status_;
      as_.publishFeedback(feedback_);
    }
    
    void goalCB() {
      goal_ = as_.acceptNewGoal()->pose;
      calcDistance();
      if(initial_distance_ == -1) {
        initial_x_ = x_;
        initial_y_ = y_;
        initial_z_ = z_;
        initial_distance_ = distance_;
      }
      ROS_INFO("New goal accepted!");
    }
    
    void preempCB() {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }
    
    void getPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
      tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf::Matrix3x3 m(q);
      m.getRPY(roll_, pitch_, yaw_);

      x_ = msg->pose.position.x;
      y_ = msg->pose.position.y;
      z_ = msg->pose.position.z;

      this->taskToBeDone();
    }
    
  protected:
    
    // mandatory variables
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actlib::QuadAction> as_;
    std::string action_name_;
    actlib::QuadFeedback feedback_;
    actlib::QuadResult result_;
    ros::Subscriber sub_;
    ros::Publisher pub1_;
    ros::Publisher pub2_;
    
    // initial variables
    float initial_distance_, initial_x_, initial_y_, initial_z_;
    // current variables
    double roll_, pitch_, yaw_, x_, y_, z_, distance_, status_;
    // goal variables
    geometry_msgs::Vector3 goal_;
    // to publish variables
    geometry_msgs::Vector3Stamped attitude_;
    std_msgs::Float32 thrust_;
    
};

int main(int argc, char** argv) {
  ros::Time::init();
  ros::init(argc, argv, "quad");
  ros::Rate loop_rate(20);
  
  QuadAction quad(ros::this_node::getName());
  while(ros::ok()) {
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  return 0;
}