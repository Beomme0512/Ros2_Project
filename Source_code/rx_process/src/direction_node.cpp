#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_interface/msg/time_cycle.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using std::placeholders::_1;
using TimeCycleMsg = robot_interface::msg::TimeCycle;
using StringMsg = std_msgs::msg::String;
using BoolMsg = std_msgs::msg::Bool;

class DirectionNode : public rclcpp::Node
{
public:
  DirectionNode()
  : Node("direction_node")
  {
    publisher_ = this->create_publisher<StringMsg>("Direction", 10);
    Pub_boost_ = this->create_publisher<BoolMsg>("Spdboost", 10);

    subscriber_ = this->create_subscription<TimeCycleMsg>(
      "TimeCycle", 10,
      std::bind(&DirectionNode::timeCycleCallback, this, _1));

    configureTerminal();
    RCLCPP_INFO(this->get_logger(), "DirectionNode started.");
  }

  ~DirectionNode()
  {
    resetTerminal();
  }

private:
  void timeCycleCallback(const TimeCycleMsg::SharedPtr msg)
  {
    if (msg->cyc100ms_b) {
      mainLoop();
    }

  }

  void mainLoop()
  {
    std::string direction = "None";
    char key = getKeyPress();

    if (key != '\0') {
      switch (key) {
        case 'A': direction = "Up"; break;
        case 'B': direction = "Down"; break;
        case 'C': direction = "Right"; break;
        case 'D': direction = "Left"; break;
        default: direction = "None"; break;
      }
    }

    // boost 감지 로직
    if (direction != "None") {
      if (direction == last_direction_) {
        same_direction_count_++;
      } else {
        same_direction_count_ = 1;  // 새로운 방향
      }
      last_direction_ = direction;
    } else {
      same_direction_count_ = 0;
    }
    bool current_boost = same_direction_count_ >= 15;  // 3초 지속시 boost

    if (current_boost != boost_active_) {
      auto boost_msg = BoolMsg();
      boost_msg.data = current_boost;
      Pub_boost_->publish(boost_msg);
      RCLCPP_INFO(this->get_logger(), "Published Boost: %s", current_boost ? "true" : "false");
      boost_active_  = current_boost;      
    }
    if (direction != "None") {
      auto msg = StringMsg();
      msg.data = direction;
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published Direction: '%s'", direction.c_str());
    }
  }

  char getKeyPress()
  {
    char c = '\0';
    int bytesWaiting;
    ioctl(0, FIONREAD, &bytesWaiting);
    if (bytesWaiting >= 3) {
      read(0, &c, 1);
      if (c == '\033') {
        char seq[2];
        read(0, &seq[0], 1);
        read(0, &seq[1], 1);
        tcflush(STDIN_FILENO, TCIFLUSH);
        return seq[1];  // 'A', 'B', 'C', or 'D'
      }
    }
    return '\0';
  }

  void configureTerminal()
  {
    tcgetattr(STDIN_FILENO, &old_tio_);
    struct termios new_tio = old_tio_;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  }

  void resetTerminal()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
  }

  rclcpp::Publisher<StringMsg>::SharedPtr publisher_;
  rclcpp::Subscription<TimeCycleMsg>::SharedPtr subscriber_;
  rclcpp::Publisher<BoolMsg>::SharedPtr Pub_boost_;
  struct termios old_tio_;
  // 내부 상태
  std::string last_direction_;
  int same_direction_count_;
  bool boost_active_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionNode>());
  rclcpp::shutdown();
  return 0;
}