#include "drone_offboard/smart_follow_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartFollowNode>());
    rclcpp::shutdown();
    return 0;
}
