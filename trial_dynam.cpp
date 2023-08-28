#include <vector>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

using namespace std::chrono_literals;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode()
        : Node("subscriber_node")
    {
        for (int i = 1; i <= 9; ++i)
        {
            std::string topic_name = "Robot/Cable" + std::to_string(i) + "/state/displacement";
            auto subscriber = create_subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
                topic_name, 10,
                std::bind(&SubscriberNode::set_position_callback, this, std::placeholders::_1));
            subscribers_.push_back(subscriber);
        }

        set_position_publisher_ = create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
            "set_position", 10);
    }

private:
    void set_position_callback(const dynamixel_sdk_custom_interfaces::msg::SetPosition::SharedPtr msg)
    {
        // Process the received message from the set_position topics
        RCLCPP_INFO(this->get_logger(), "Received: ID = %d, Position = %d", msg->id, msg->position);

        // Publish to the "set_position" topic
        dynamixel_sdk_custom_interfaces::msg::SetPosition set_position_msg;
        set_position_msg.id = msg->id;
        set_position_msg.position = msg->position;  // Modify this value if needed
        set_position_publisher_->publish(set_position_msg);
    }

    std::vector<rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr> subscribers_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr set_position_publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Create the SubscriberNode
    auto node = std::make_shared<SubscriberNode>();

    // Spin the node in a loop to process callbacks
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

