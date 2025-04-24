#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MotionCtrlNode : public rclcpp::Node
{
public:
    MotionCtrlNode() : Node("motion_ctrl_node"),
                       linear_x_(0.0), angular_z_(0.0),
                       last_direction_(""), last_update_time_(this->now())
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        direction_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "Direction", 10,
            std::bind(&MotionCtrlNode::direction_callback, this, std::placeholders::_1));

        cycle_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "TimeCycle/Cyc5ms_b", 10,
            std::bind(&MotionCtrlNode::cycle_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MotionCtrlNode started.");
    }

private:
    void direction_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_direction_ = msg->data;

        if (current_direction_ == last_direction_) {
            auto now = this->now();
            if ((now - last_update_time_).seconds() >= 3.0) {
                sustained_direction_ = true;
            }
        } else {
            last_direction_ = current_direction_;
            last_update_time_ = this->now();
            sustained_direction_ = false;
        }
    }

    void cycle_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data || current_direction_.empty())
            return;

        // 초기화
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_x_;
        twist.angular.z = angular_z_;

        if (current_direction_ == "Right") {
            twist.linear.x += sustained_direction_ ? 0.3 : 0.1;
        } else if (current_direction_ == "Left") {
            twist.linear.x -= sustained_direction_ ? 0.3 : 0.1;
        } else if (current_direction_ == "Up") {
            twist.angular.z += 0.1;
        } else if (current_direction_ == "Down") {
            twist.angular.z -= 0.1;
        }

        cmd_vel_publisher_->publish(twist);
        linear_x_ = twist.linear.x;
        angular_z_ = twist.angular.z;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direction_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cycle_subscriber_;

    std::string current_direction_;
    std::string last_direction_;
    rclcpp::Time last_update_time_;
    bool sustained_direction_ = false;

    double linear_x_;
    double angular_z_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionCtrlNode>());
    rclcpp::shutdown();
    return 0;
}