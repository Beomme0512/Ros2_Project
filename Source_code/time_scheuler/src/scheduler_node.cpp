#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class SchedulerNode : public rclcpp::Node
{
public:
    SchedulerNode() : Node("scheduler_node")
    {
        pub_5ms_ = this->create_publisher<std_msgs::msg::Bool>("TimeCycle/Cyc5ms_b", 10);
        pub_10ms_ = this->create_publisher<std_msgs::msg::Bool>("TimeCycle/Cyc10ms_b", 10);

        timer_5ms_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&SchedulerNode::publish_5ms, this));

        timer_10ms_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SchedulerNode::publish_10ms, this));

        RCLCPP_INFO(this->get_logger(), "SchedulerNode started with 5ms and 10ms timers.");
    }

private:
    void publish_5ms()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pub_5ms_->publish(msg);
    }

    void publish_10ms()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pub_10ms_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_5ms_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_10ms_;
    rclcpp::TimerBase::SharedPtr timer_5ms_;
    rclcpp::TimerBase::SharedPtr timer_10ms_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SchedulerNode>());
    rclcpp::shutdown();
    return 0;
}