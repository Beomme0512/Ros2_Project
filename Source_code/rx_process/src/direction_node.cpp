#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class DirectionNode : public rclcpp::Node
{
public:
    DirectionNode() : Node("direction_node")
    {
        // Publisher 생성
        direction_publisher_ = this->create_publisher<std_msgs::msg::String>("Direction", 10);

        // Subscriber 생성
        time_cycle_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "TimeCycle/Cyc10ms_b", 10,
            std::bind(&DirectionNode::timeCycleCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "DirectionNode has been started.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr time_cycle_subscriber_;
    bool cyc10ms_active_ = false;

    // Function_01: Cyc10ms_b 수신 처리
    void timeCycleCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        cyc10ms_active_ = msg->data;

        if (cyc10ms_active_)
        {
            handleKeyboardInput();
        }
    }

    // Function_02 + Function_03 + Function_04: 키보드 입력 처리 및 Publish + 터미널 출력
    void handleKeyboardInput()
    {
        std::string direction = readKeyboardDirection();

        // Publish Message 생성
        std_msgs::msg::String direction_msg;
        direction_msg.data = direction;

        // Publish 수행
        direction_publisher_->publish(direction_msg);

        // Function_04: Debug 출력
        RCLCPP_INFO(this->get_logger(), "Published Direction: '%s'", direction.c_str());
    }

    // 키보드 입력 읽기
    std::string readKeyboardDirection()
    {
        int key = getKeyPress();
        switch (key)
        {
            case 65:  // 위 방향키
                return "Up";
            case 66:  // 아래 방향키
                return "Down";
            case 67:  // 오른쪽 방향키
                return "Right";
            case 68:  // 왼쪽 방향키
                return "Left";
            default:
                return "None";
        }
    }

    // 비차단 방식 키보드 입력
    int getKeyPress()
    {
        struct termios oldt, newt;
        int ch;
        int oldf;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        ch = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if (ch != EOF)
        {
            // 방향키 입력 처리
            if (ch == 27)
            {
                getchar(); // '[' 버림
                ch = getchar(); // 방향 코드 (A, B, C, D)
            }
            return ch;
        }
        else
        {
            return -1;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionNode>());
    rclcpp::shutdown();
    return 0;
}