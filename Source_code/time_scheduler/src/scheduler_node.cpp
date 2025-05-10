#include "rclcpp/rclcpp.hpp"
#include "robot_interface/msg/time_cycle.hpp"

using TimeCycleMsg = robot_interface::msg::TimeCycle;

class SchedulerNode : public rclcpp::Node
{
public:
  SchedulerNode()
  : Node("schedulernode"), cnt20ms_(0), cnt100ms_(0), cnt500ms_(0)
  {
    publisher_ = this->create_publisher<TimeCycleMsg>("TimeCycle", 10);

    timer_5ms_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&SchedulerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "schedulernode started.");
  }

private:
  void timerCallback()
  {
    TimeCycleMsg msg;
    
    // Function_01: 5ms 주기 신호
    msg.cyc5ms_b = true;

    // Function_02: 카운터 증가
    cnt20ms_++;
    cnt100ms_++;
    cnt500ms_++;

    // Function_03: 20ms (4회 마다)
    if (cnt20ms_ >= 4) {
      msg.cyc20ms_b = true;
      cnt20ms_ = 0;
    }

    // Function_04: 100ms (20회 마다)
    if (cnt100ms_ >= 20) {
      msg.cyc100ms_b = true;
      cnt100ms_ = 0;
    }

    // Function_05: 500ms (100회 마다)
    if (cnt500ms_ >= 100) {
      msg.cyc500ms_b = true;
      cnt500ms_ = 0;
    }

    publisher_->publish(msg);
    // 디버깅: 모든 주기 상태 한 줄 출력  
    RCLCPP_INFO(this->get_logger(), 
     "[TimeCycle] 5ms: %d | 20ms: %d | 100ms: %d | 500ms: %d",
     msg.cyc5ms_b, msg.cyc20ms_b, msg.cyc100ms_b, msg.cyc500ms_b);
  }

  rclcpp::Publisher<TimeCycleMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_5ms_;
  

  
  // Internal counters
  int cnt20ms_;
  int cnt100ms_;
  int cnt500ms_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SchedulerNode>());
  rclcpp::shutdown();
  return 0;
}
