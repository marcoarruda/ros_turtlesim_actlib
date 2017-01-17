#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <actlib/QuadAction.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class QuadAction {
  
  public:
  
    QuadAction(std::string name) : as_(nh_, name, false), action_name_(name) {
      ROS_INFO("name: %s", name.c_str());
      as_.registerGoalCallback(boost::bind(&QuadAction::goalCB, this));
      as_.registerPreemptCallback(boost::bind(&QuadAction::preempCB, this));
      
      initial_distance_ = -1;
      stay_.x = 0;
      stay_.y = 0;
      stay_.z = -1;
      kp_[0] = 0.07; kp_[1] = 0.07; kp_[2] = 0.50;
      kd_[0] = 0.05; kd_[1] = 0.05; kd_[2] = 0.05;
      ff_[0] = 0; ff_[1] = 0; ff_[2] = 1.2 * 9.81;
      
      sub1_ = nh_.subscribe("/vrep/pose", 1, &QuadAction::getPose, this);
      sub2_ = nh_.subscribe("/vrep/odometry", 1, &QuadAction::getVelocity, this);
      pub1_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/vrep/cmd_attitude", 1);
      pub2_ = nh_.advertise<std_msgs::Float32>("/vrep/cmd_thrust", 1);
      as_.start();
    }
    
    ~QuadAction(void) {
    }

    // Tasks
    void taskToBeDone() {
      // stay
      if(!as_.isActive()) {
        //stay();
        return;
      }

      calcDistance();
      
      // arrived
      /*
      if(distance_ < 0.05) {
        arrived();
        return;
      }
      */

      // goTo
      goTo();
    }
    void calcDistance() {
        distance_ = sqrt(pow(pose_.z - goal_.z, 2) + pow(pose_.x - goal_.x, 2) + pow(pose_.y - goal_.y, 2));
    }
    void stay() {
        // stay where you are
        e_[0] = stay_.x - pose_.x;
        e_[1] = stay_.y - pose_.y;
        e_[2] = stay_.z - pose_.z;
        de_[0] = 0;//-velocity_.x;
        de_[1] = 0;//-velocity_.y;
        de_[2] = 0;//-velocity_.z;
        attitude_.vector.x = ff_[0] + (kp_[0]*e_[0] + kd_[0]*de_[0]);
        attitude_.vector.y = ff_[1] - (kp_[1]*e_[1] + kd_[1]*de_[1]);
        pub1_.publish(attitude_);
        thrust_.data = ff_[2] - (kp_[2]*e_[2] + kd_[2]*de_[2]);
        pub2_.publish(thrust_);
    }
    void goTo() {
        // math for velocity command publishing
        e_[0] = goal_.x - pose_.x;
        e_[1] = goal_.y - pose_.y;
        e_[2] = goal_.z - pose_.z;
        de_[0] = -velocity_.x;
        de_[1] = -velocity_.y;
        de_[2] = -velocity_.z;
  
        attitude_.vector.x = ff_[0] + (kp_[0]*e_[0] + kd_[0]*de_[0]);
        attitude_.vector.y = ff_[1] - (kp_[1]*e_[1] + kd_[1]*de_[1]);
        pub1_.publish(attitude_);
        thrust_.data = ff_[2] - (kp_[2]*e_[2] + kd_[2]*de_[2]);
        pub2_.publish(thrust_);
        status_ = 100 * (1 - (distance_ / initial_distance_));
        
        // publish feedback
        feedback_.pose.x = pose_.x;
        feedback_.pose.y = pose_.y;
        feedback_.pose.z = pose_.z;
        feedback_.distance = distance_;
        feedback_.status = status_;
        as_.publishFeedback(feedback_);
    }
    void arrived() {
        result_.pose.x = pose_.x;
        result_.pose.y = pose_.y;
        result_.pose.z = pose_.z;
        result_.distance = distance_;
        result_.status = status_;

        stay_.x = pose_.x;
        stay_.y = pose_.y;
        stay_.z = pose_.z;

        attitude_.vector.x = 0;
        attitude_.vector.y = 0;
        attitude_.vector.z = 0;
        pub1_.publish(attitude_);
        thrust_.data = ff_[2];
        pub2_.publish(thrust_);

        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
    }
    
    // ActionLib Callbacks
    void goalCB() {
      goal_ = as_.acceptNewGoal()->pose;
      calcDistance();
      if(initial_distance_ == -1) {
        initial_x_ = pose_.x;
        initial_y_ = pose_.y;
        initial_z_ = pose_.z;
        initial_distance_ = distance_;
      }
      ROS_INFO("New goal accepted!");
    }
    void preempCB() {
      stay_.x = pose_.x;
      stay_.y = pose_.y;
      stay_.z = pose_.z;
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }
    
    // Subscriptions callbacks
    void getPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
      /*
      tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf::Matrix3x3 m(q);
      m.getRPY(roll_, pitch_, yaw_);
      */

      pose_.x= msg->pose.position.x;
      pose_.y= msg->pose.position.y;
      pose_.z= msg->pose.position.z;

      this->taskToBeDone();
    }
    void getVelocity(const nav_msgs::Odometry::ConstPtr& msg) {
      velocity_.x = msg->twist.twist.linear.x;
      velocity_.y = msg->twist.twist.linear.y;
      velocity_.z = msg->twist.twist.linear.z;
    }
    
  protected:
    
    // mandatory variables
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actlib::QuadAction> as_;
    std::string action_name_;
    actlib::QuadFeedback feedback_;
    actlib::QuadResult result_;
    ros::Subscriber sub1_, sub2_;
    ros::Publisher pub1_, pub2_;
    
    // initial variables
    float initial_distance_, initial_x_, initial_y_, initial_z_;
    // current variables
    double roll_, pitch_, yaw_, distance_, status_;
    geometry_msgs::Vector3 pose_, velocity_;
    // goal variables
    geometry_msgs::Vector3 goal_, stay_;
    // to publish variables
    geometry_msgs::Vector3Stamped attitude_;
    std_msgs::Float32 thrust_;

    // Control parameters
    double kp_[3];
    double kd_[3];
    double ff_[3];
    double e_[3], de_[3];
    
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