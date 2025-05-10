#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "robot_interface/msg/time_cycle.hpp"  // TimeCycle 메시지 포함

class UssRxNode : public rclcpp::Node
{
public:
  UssRxNode()
  : Node("uss_rx_node"), range_fnt_(0.0), latest_range_(0.0), cyc20ms_flag_(false)
  {
    using std::placeholders::_1;

    // Subscribers
    timecycle_sub_ = this->create_subscription<robot_interface::msg::TimeCycle>(
      "TimeCycle", 10, std::bind(&UssRxNode::timecycle_callback, this, _1));

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/ultrasonic", 10, std::bind(&UssRxNode::laser_callback, this, _1));

    // Publisher
    uss_pub_ = this->create_publisher<std_msgs::msg::Float32>("uss_signal", 10);
  }

private:
  void timecycle_callback(const robot_interface::msg::TimeCycle::SharedPtr msg)
  {
    if (msg->cyc20ms_b) {  // TimeCycle 메시지의 cyc20ms_b 필드 사용
      // Function_01: Cyc20ms_b 트리거로 노드 활성화
      // Function_02: range_fnt 계산
      range_fnt_ = 0.5f * (latest_range_ - range_fnt_) + range_fnt_;
      range_fnt_ = std::max(0.0f, range_fnt_);

      // Function_03: uss_signal 퍼블리시
      auto msg_out = std_msgs::msg::Float32();
      msg_out.data = range_fnt_;
      uss_pub_->publish(msg_out);

      RCLCPP_INFO(this->get_logger(), "Published uss_signal: %.3f", range_fnt_);
    }
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "laser_callback triggered");
    if (!msg->ranges.empty()) {
      latest_range_ = msg->ranges[0];
      RCLCPP_INFO(this->get_logger(), "raw uss_signal: %.3f", latest_range_);
    } else {
      RCLCPP_WARN(this->get_logger(), "LaserScan message has empty ranges.");
    }
  }

  // Subscribers & Publisher
  rclcpp::Subscription<robot_interface::msg::TimeCycle>::SharedPtr timecycle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr uss_pub_;

  // Internal Variables
  float range_fnt_;
  float latest_range_;
  bool cyc20ms_flag_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UssRxNode>());
  rclcpp::shutdown();
  return 0;
}