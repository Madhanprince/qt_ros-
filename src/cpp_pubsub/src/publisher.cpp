#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msg/msg/string.hpp>

class MinimalPublisher : public rclcpp::Node
{
    MinimalPublisher() : Node("minimal_publisher"),count(0){

    }
