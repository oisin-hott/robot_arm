#include "arm_motion_node.hpp"
#include <string>
#include <std_msgs/msg/empty.hpp>

ArmMotionNode::ArmMotionNode(const rclcpp::NodeOptions & options)
 : Node("arm_motion_node", options)                                       // CHANGED: no more move_group_ in initializer list
    {
        RCLCPP_INFO(this->get_logger(), "ArmMotionNode started");
    }

// ---------------- INIT ----------------                                 // CHANGED: new method
void ArmMotionNode::init()
{
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");                                       // shared_from_this is safe here

    gripper_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "Gripper");    

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&ArmMotionNode::controlLoop, this)
    );
    move_group_->setMaxVelocityScalingFactor(0.5);                        // CHANGED: . -> ->
    move_group_->setMaxAccelerationScalingFactor(0.5);   
    move_group_->setPlannerId("stomp_moveit/StompPlanner");
    move_group_->setPlanningTime(15.0);                 // CHANGED: . -> ->
    this->get_parameter("home", home_);
    this->get_parameter("pickup", pickup_);
    this->get_parameter("dropoff", dropoff_);
    this->get_parameter("inspection", inspection_);
    this->get_parameter("place", place_);
    this->get_parameter("return", return_);




    wrist_control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wrist_commands",
        10
    );
    attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/box/attach", 10);
    detach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/box/detach", 10);
    // at the end of init(), after creating the publishers:
    detach_pub_->publish(std_msgs::msg::Empty());   // break the initial auto-attach
    loadInitialSequence();
}

// ---------------- CONTROL LOOP ----------------
void ArmMotionNode::controlLoop()
{
    if (task_queue_.empty()) {
        return;
    }

    Task next;
    {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    next = task_queue_.front();
    task_queue_.pop();
    }
    RCLCPP_INFO(this->get_logger(), "Executing task: %d", next);

    switch (next) {
        case GRIP_OPEN:
            gripper_group_->setNamedTarget("Open");
            if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_WARN(this->get_logger(), "Gripper open failed");
            }
                detach_pub_->publish(std_msgs::msg::Empty());
            return;
        case GRIP_CLOSE:
            gripper_group_->setNamedTarget("Closed");
            if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS)
                RCLCPP_WARN(this->get_logger(), "Gripper close failed");
            return;
        case GRIP_GRAB:
            gripper_group_->setNamedTarget("Grab");
            if (gripper_group_->move() != moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_WARN(this->get_logger(), "Gripper grab failed");
            }
                attach_pub_->publish(std_msgs::msg::Empty());
            return;
        default:
            break;  // arm tasks fall through to pose handling below
    }
        // select target
    std::vector<double>* target = nullptr;
    switch (next) {
        case HOME: target = &home_;
            break;
        case INSPECTION: target = &inspection_;
            break;
        case PICKUP: target = &pickup_;
            break;
        case DROPOFF: target = &dropoff_;
            break;
        case PLACE: target = &place_;
            break;
        case RETURN: target = &return_;
            break;   

    }
    // MoveIt2 inline:
    if (target && target->size() >= 7)
    {   
        geometry_msgs::msg::Pose pose;
        pose.position.x = (*target)[0];
        pose.position.y = (*target)[1];
        pose.position.z = (*target)[2];
        pose.orientation.x = (*target)[3];
        pose.orientation.y= (*target)[4];
        pose.orientation.z = (*target)[5];
        pose.orientation.w =(*target)[6];

        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_->execute(plan);
            std_msgs::msg::Float64MultiArray wrist_msg;
            wrist_msg.data = {(*target)[4], (*target)[5]};
            wrist_control_pub_->publish(wrist_msg);

        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Plan Failed");
        }


    }
    else if (target)
    {
        RCLCPP_ERROR(this->get_logger(), "Pose vector has %zu elements, expected 6", target->size());
    }
}
// ---------------- ----------------
void ArmMotionNode::loadInitialSequence()
{
    //task_queue_.push(HOME);
    //task_queue_.push(RETURN);
    task_queue_.push(INSPECTION);
    task_queue_.push(PICKUP);
    task_queue_.push(GRIP_GRAB);
    task_queue_.push(INSPECTION);
   //task_queue_.push(RETURN);
    task_queue_.push(DROPOFF);
    task_queue_.push(PLACE);
    task_queue_.push(GRIP_OPEN);
    task_queue_.push(DROPOFF);
    task_queue_.push(RETURN);
    task_queue_.push(GRIP_CLOSE);

    RCLCPP_INFO(this->get_logger(), "Loaded 4 tasks");
}
// ---------------- MAIN ----------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<ArmMotionNode>(options);
    node->init();                                                         // CHANGED: call init after make_shared
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}