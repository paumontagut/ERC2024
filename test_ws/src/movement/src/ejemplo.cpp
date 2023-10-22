#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ejemplo_cpp");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

