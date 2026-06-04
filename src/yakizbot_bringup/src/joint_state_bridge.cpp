#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateBridge : public rclcpp::Node
{
public:
    JointStateBridge() : Node("joint_state_bridge")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/encoder_positions",
            10,
            std::bind(&JointStateBridge::callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            10
        );

        joint_names_ = {
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint"
        };

        RCLCPP_INFO(this->get_logger(), "JointState Bridge Node Started");
    }

private:

    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4)
        {
            RCLCPP_WARN(this->get_logger(), "Not enough encoder data");
            return;
        }

        sensor_msgs::msg::JointState joint_msg;

        joint_msg.header.stamp = this->now();
        joint_msg.header.frame_id = "base_link";
        
        joint_msg.name = joint_names_;
        joint_msg.position = {
            msg->data[0],
            msg->data[1],
            msg->data[2],
            msg->data[3]
        };

        publisher_->publish(joint_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    std::vector<std::string> joint_names_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}