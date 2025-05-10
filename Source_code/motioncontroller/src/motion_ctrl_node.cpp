#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_interface/msg/time_cycle.hpp"
#include <algorithm>  // std::clamp

using std::placeholders::_1;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;

class MotionCtrlNode : public rclcpp::Node
{
public:
  MotionCtrlNode()
  : Node("motion_ctrl_node"), cyc5ms_ready_(false), cyc1050ms_ready_(false),last_direction_("None")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    time_sub_ = this->create_subscription<robot_interface::msg::TimeCycle>(
      "TimeCycle", 10, std::bind(&MotionCtrlNode::timeCallback, this, _1));

    dir_sub_ = this->create_subscription<std_msgs::msg::String>(
      "Direction", 10, std::bind(&MotionCtrlNode::dirCallback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&MotionCtrlNode::mainLoop, this));

    RCLCPP_INFO(this->get_logger(), "MotionCtrlNode started.");
  }

private:
  void timeCallback(const robot_interface::msg::TimeCycle::SharedPtr msg)
  {
    cyc5ms_ready_ = msg->cyc5ms_b;
    cyc1050ms_ready_ = msg->cyc500ms_b;
  }

  void dirCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string dir = msg->data;
    if(dir != "None") {
      if (dir == "Right" || dir == "Left") {
        if (dir == last_direction_) {
          // Same direction → check duration
          auto now = steady_clock::now();
          auto duration = duration_cast<seconds>(now - last_direction_start_);
          if (duration.count() >= 3) { // 실제 카운트는 
            sustained_direction_ = true;
          }
        } else {
          last_direction_start_ = steady_clock::now();
          sustained_direction_ = false;
        }
      } else {
        sustained_direction_ = false;
      }

      if (current_direction_ == "Up") {
        angular_z_ += 0.1;
      } else if (current_direction_ == "Down") {
          angular_z_-= 0.1;
      } else if (current_direction_ == "Right") {
          linear_x_ += 0.1;
        if (sustained_direction_) linear_x_ += 0.3;
      } else if (current_direction_ == "Left") {
          linear_x_ -= 0.1;
        if (sustained_direction_) linear_x_-= 0.3;
      }
    }
    

    last_direction_ = dir;
    current_direction_ = dir;
    dir = "None";
  }

  void mainLoop()
  {
    if (!cyc5ms_ready_) return;

    geometry_msgs::msg::Twist msg;

    linear_x_ = std::clamp(linear_x_, -3.0, 3.0);
    angular_z_ = std::clamp(angular_z_, -1.0, 1.0); 

    msg.linear.x = linear_x_;
    msg.angular.z = angular_z_;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published Twist: linear.x=%.2f angular.z=%.2f",
                msg.linear.x, msg.angular.z);

    cyc5ms_ready_ = false;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<robot_interface::msg::TimeCycle>::SharedPtr time_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dir_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool cyc5ms_ready_;
  bool cyc1050ms_ready_;

  std::string current_direction_;
  std::string last_direction_;
  steady_clock::time_point last_direction_start_;
  bool sustained_direction_ = false;

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