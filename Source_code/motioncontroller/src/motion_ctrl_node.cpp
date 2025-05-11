#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_interface/msg/time_cycle.hpp"
#include <algorithm>  // std::clamp

using std::placeholders::_1;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;
using BoolMsg = std_msgs::msg::Bool;
class MotionCtrlNode : public rclcpp::Node
{
public:
  MotionCtrlNode()
  : Node("motion_ctrl_node"), current_dir("None")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    time_sub_ = this->create_subscription<robot_interface::msg::TimeCycle>(
      "TimeCycle", 10, std::bind(&MotionCtrlNode::timeCallback, this, _1));

    dir_sub_ = this->create_subscription<std_msgs::msg::String>(
      "Direction", 10, std::bind(&MotionCtrlNode::dirCallback, this, _1));
     
    boost_sub_ = this->create_subscription<BoolMsg>(
      "Spdboost", 10, std::bind(&MotionCtrlNode::boostCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "MotionCtrlNode started.");
  }

private:
  void dirCallback(const std_msgs::msg::String::SharedPtr msg){
    current_dir = msg->data;
  }
  void boostCallback(const BoolMsg::SharedPtr msg){
    boost_act_ = msg->data;
  }
  
  void timeCallback(const robot_interface::msg::TimeCycle::SharedPtr msg)
  {
    if(msg->cyc5ms_b){
      if(msg->cyc100ms_b)rxsigprocess();
      mainCtrlLoop();
    } 
  }

  void rxsigprocess()
  {
    if (current_dir == "Up") {
      angular_z_ += 0.1;
    } else if (current_dir == "Down") {
        angular_z_-= 0.1;
    } else if (current_dir == "Right") {
      if (boost_act_)linear_x_ += 0.2; // Boosted speed
      else linear_x_ += 0.1;
    } else if (current_dir == "Left") {
      if (boost_act_)linear_x_ -= 0.2; // Boosted speed
      else linear_x_ -= 0.1;
    }
    previous_dir = current_dir; // Store the last direction
    current_dir = "None"; // Reset direction after processing
  }

  void mainCtrlLoop()
  {
    geometry_msgs::msg::Twist msg;

    linear_x_ = std::clamp(linear_x_, -3.0, 3.0);
    angular_z_ = std::clamp(angular_z_, -1.0, 1.0); 

    msg.linear.x = linear_x_;
    msg.angular.z = angular_z_;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published Twist: linear.x=%.2f angular.z=%.2f",
                msg.linear.x, msg.angular.z);
  }



  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<robot_interface::msg::TimeCycle>::SharedPtr time_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dir_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr boost_sub_;

  std::string current_dir;
  std::string previous_dir;
  bool boost_act_;

  double linear_x_;
  double angular_z_;
  
};
  
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionCtrlNode>());
  rclcpp::shutdown();
  return 0;
}