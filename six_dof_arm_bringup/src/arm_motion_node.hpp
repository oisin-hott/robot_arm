#pragma once     //include guard of being called multiple times
#include <rclcpp/rclcpp.hpp>  //ROS2 Node
#include <std_msgs/msg/float64_multi_array.hpp>  //message type
#include <moveit/move_group_interface/move_group_interface.hpp> //moveit
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <functional>
#include <memory>   // CHANGED: for std::unique_ptr
#include <std_msgs/msg/empty.hpp>


class   ArmMotionNode : public rclcpp::Node
{
    public:
    ArmMotionNode(const rclcpp::NodeOptions & options);
    void init();                                                          // CHANGED: new method

    private:
    enum Task {HOME, INSPECTION, PICKUP, DROPOFF, PLACE, RETURN, GRIP_OPEN, GRIP_GRAB, GRIP_CLOSE}; //create task
    std::queue<Task> task_queue_;
    std::mutex queue_mutex_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wrist_control_pub_;
    std::vector<double> home_;
    std::vector<double> pickup_;
    std::vector<double> dropoff_;
    std::vector<double> inspection_;
    std::vector<double> place_;
    std::vector<double> return_;
    //MoveIt SetUP
    static const std::string PLANNING_GROUP;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;  // CHANGED: was a value, now a unique_ptr
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;

    void controlLoop();
    void loadInitialSequence();

};