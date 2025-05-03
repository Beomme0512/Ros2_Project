#include "rclcpp/rclcpp.hpp"
#include "robot_interface/msg/time_cycle.hpp"

using std::placeholders::_1;
using TimeCycleMsg = robot_interface::msg::TimeCycle;

class SchedulerNode : public rclcpp::Node
{
public:
  SchedulerNode()
  : Node("scheduler_node")
  {
    publisher_ = this->create_publisher<TimeCycleMsg>("TimeCycle", 10);

    timer_5ms_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&SchedulerNode::timer5msCallback, this));

    timer_20ms_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SchedulerNode::timer20msCallback, this));

    timer_100ms_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SchedulerNode::timer100msCallback, this));

    RCLCPP_INFO(this->get_logger(), "SchedulerNode has started.");
  }

private:
  void timer5msCallback()
  {
    msg_.cyc5ms_b = true;
    publishMsg();
  }

  void timer20msCallback()
  {
    msg_.cyc20ms_b = true;
    publishMsg();
  }

  void timer100msCallback()
  {
    msg_.cyc100ms_b = true;
    publishMsg();
  }

  void publishMsg()
  {
    publisher_->publish(msg_);
    // 각 주기마다 발행 후 false로 초기화
    msg_.cyc5ms_b = false;
    msg_.cyc20ms_b = false;
    msg_.cyc100ms_b = false;
  }

  rclcpp::Publisher<TimeCycleMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_5ms_;
  rclcpp::TimerBase::SharedPtr timer_20ms_;
  rclcpp::TimerBase::SharedPtr timer_100ms_;
  TimeCycleMsg msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SchedulerNode>());
  rclcpp::shutdown();
  return 0;
}