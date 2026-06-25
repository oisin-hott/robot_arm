#pragma once     //include guard of being called multiple times

#include <rclcpp/rclcpp.hpp>  //ROS2 Node
#include <std_msgs/msg/float64_multi_array.hpp>  //message type


class DifferentialMixer : public rclcpp::Node
{
    public:
    DifferentialMixer();

    private:

    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    double joint_5_angle_;
    double joint_6_angle_;

    const double gear_ratio_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

};