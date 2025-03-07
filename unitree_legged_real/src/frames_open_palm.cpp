#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode() : Node("node_ros2_frames_open_palm")
    {
        // Publisher for robot   commands
        pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

        // Subscriber to the "active_gesture" topic
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "active_gesture", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received gesture: '%s'", msg->data.c_str());
                this->current_gesture_ = msg->data;

            });

        // Timer to repeatedly execute the main motion loop at 500Hz (2ms per loop)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), // 2ms loop rate
            std::bind(&RobotControlNode::motion_loop, this));
    }

private:
    // Motion execution function, called at 500Hz

void motion_loop()
{
    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.level_flag = HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gait_type = 0;
    high_cmd_ros.speed_level = 0;
    high_cmd_ros.foot_raise_height = 0;
    high_cmd_ros.body_height = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yaw_speed = 0.0f;
    high_cmd_ros.reserve = 0;

    int target_body_height = 0; // Default neutral posture

    if (current_gesture_ == "down")
    {
        RCLCPP_INFO(this->get_logger(), "Executing 'down' command.");
        high_cmd_ros.velocity[0] = 0;
        high_cmd_ros.velocity[1] = 0;
        high_cmd_ros.mode = 5;
    }

    else if (current_gesture_ == "up") {
        RCLCPP_INFO(this->get_logger(), "Executing 'up' command.");
        
        high_cmd_ros.velocity[0] = 0;
        high_cmd_ros.velocity[1] = 0;

        high_cmd_ros.mode = 6;
    }
    else if (current_gesture_ == "right") {
        RCLCPP_INFO(this->get_logger(), "Executing 'going right' command.");
        high_cmd_ros.mode = 2;
        high_cmd_ros.gait_type = 1;
        high_cmd_ros.velocity[0] = 0; // -1  ~ +1
        high_cmd_ros.velocity[1] = -0.4f; // Goes right
        high_cmd_ros.foot_raise_height = 0.1;
    }
    else if (current_gesture_ == "left") {
        RCLCPP_INFO(this->get_logger(), "Executing 'going left' command.");
        high_cmd_ros.mode = 2;
        high_cmd_ros.gait_type = 1;
        high_cmd_ros.velocity[0] = 0; // -1  ~ +1
        high_cmd_ros.velocity[1] = 0.4f; // Goes left
        high_cmd_ros.foot_raise_height = 0.1;
    }
    else if (current_gesture_ == "forward") {
        RCLCPP_INFO(this->get_logger(), "Executing 'going forward' command.");
        high_cmd_ros.mode = 2;
        high_cmd_ros.gait_type = 1;
        high_cmd_ros.velocity[0] = 0.4f; // Goes forward
        high_cmd_ros.velocity[1] = 0; // -1  ~  +1
        high_cmd_ros.foot_raise_height = 0.1;
    }
    else if (current_gesture_ == "back") {
        RCLCPP_INFO(this->get_logger(), "Executing 'going backward' command.");
        high_cmd_ros.mode = 2;
        high_cmd_ros.gait_type = 1;
        high_cmd_ros.velocity[0] = -0.4f; // Goes backward
        high_cmd_ros.velocity[1] = 0; // -1  ~ +1
        high_cmd_ros.foot_raise_height = 0.1;
    }

    else if (current_gesture_ == "hand") {
        RCLCPP_INFO(this->get_logger(), "Executing 'hand' command.");
        high_cmd_ros.velocity[0] = 0;
        high_cmd_ros.velocity[1] = 0;

        // set the mode to 1 first
        high_cmd_ros.mode = 1;
        pub_ -> publish(high_cmd_ros);


        high_cmd_ros.mode = 11;
    }



    else  {
        RCLCPP_WARN(this->get_logger(), "No valid gesture detected. Keeping neutral state.");
        
        // Do nothing if gesture is invalid
        high_cmd_ros.velocity[0] = 0;
        high_cmd_ros.velocity[1] = 0;

        return; // Do nothing if gesture is invalid
    }

    // Set the body height and publish the command
    high_cmd_ros.body_height = target_body_height;
    pub_->publish(high_cmd_ros);

    // Update the last body height and last executed gesture
    last_body_height_ = target_body_height;
    last_executed_gesture_ = current_gesture_;
}


    // ROS2 interfaces
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Current detected gesture
    std::string current_gesture_ = "";

    // Last executed gesture
    std::string last_executed_gesture_ = "";  // Tracks the last gesture to avoid repeated execution

    float last_body_height_ = 0.0;  // Initial body height (assumed neutral posture)

};

int main(int argc, char **argv)
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Create the robot control node
    auto node = std::make_shared<RobotControlNode>();

    // Spin the node to process callbacks
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
