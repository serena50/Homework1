#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ArmControllerNode : public rclcpp::Node
{
public:
    ArmControllerNode() : Node("arm_controller_node")
    {
        // Crea subscriber per joint_states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&ArmControllerNode::joint_state_callback, this, std::placeholders::_1));

        // Crea publisher per position_controller/commands
        position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(), "Arm controller node has been started");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Stampa le posizioni correnti dei giunti
        RCLCPP_INFO(this->get_logger(), "Current joint positions:");
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "%s: %f", msg->name[i].c_str(), msg->position[i]);
        }
    }

    void send_position_command(const std::vector<double>& positions)
    {
        auto command_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        command_msg->data = positions;
        position_pub_->publish(std::move(command_msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControllerNode>());
    rclcpp::shutdown();
    return 0;
}
