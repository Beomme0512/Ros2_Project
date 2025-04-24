#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class DirectionNode : public rclcpp::Node
{
public:
    DirectionNode() : Node("direction_node")
    {
        direction_publisher_ = this->create_publisher<std_msgs::msg::String>("Direction", 10);

        cycle_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "TimeCycle/Cyc10ms_b",
            10,
            std::bind(&DirectionNode::cycle_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "DirectionNode initialized.");
    }

private:
    void cycle_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            std::string key = get_key_press();
            if (!key.empty())
            {
                std_msgs::msg::String direction_msg;
                direction_msg.data = key;
                direction_publisher_->publish(direction_msg);
                RCLCPP_INFO(this->get_logger(), "Published direction: %s", key.c_str());
            }
        }
    }

    std::string get_key_press()
    {
        struct termios oldt, newt;
        int oldf;
        char ch;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        std::string direction;

        if (read(STDIN_FILENO, &ch, 1) > 0)
        {
            if (ch == '\033')
            {
                char seq[2];
                if (read(STDIN_FILENO, &seq[0], 1) > 0 &&
                    read(STDIN_FILENO, &seq[1], 1) > 0)
                {
                    switch (seq[1])
                    {
                    case 'A': direction = "Up"; break;
                    case 'B': direction = "Down"; break;
                    case 'C': direction = "Right"; break;
                    case 'D': direction = "Left"; break;
                    }
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        return direction;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cycle_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionNode>());
    rclcpp::shutdown();
    return 0;
}