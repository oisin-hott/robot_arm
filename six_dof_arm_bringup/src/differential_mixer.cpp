#include "differential_mixer.hpp" //header file


DifferentialMixer::DifferentialMixer()
: Node("differential_mixer"), joint_5_angle_(0.0), joint_6_angle_(0.0), gear_ratio_(4.0)
{

    sub_ =this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wrist_commands",
        10,
        std::bind(&DifferentialMixer::callback, this, std::placeholders::_1)
    );

    pub_= this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motor_controller_placeholder",
        10
    );
}

void DifferentialMixer::callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg
    )
{

    if (msg->data.size() >= 2)
    {
        joint_5_angle_ = msg->data[0];
        joint_6_angle_ = msg->data[1];
        
    }

    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid Message Size");
        return;
    }


    double motor_a = (joint_5_angle_ + joint_6_angle_ ) *gear_ratio_;
    double motor_b = (joint_6_angle_ - joint_5_angle_) * gear_ratio_;

    RCLCPP_INFO(this->get_logger(),"Motor A: %f, Motor B: %f", motor_a, motor_b);

    std_msgs::msg::Float64MultiArray output_msg;
    output_msg.data = {motor_a, motor_b};
    pub_->publish(output_msg);
    
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialMixer>());
    rclcpp::shutdown();
    return 0;
}

