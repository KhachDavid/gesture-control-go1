#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
    //          << "Make sure the robot is standing on the ground." << std::endl
    //          << "Press Enter to continue..." << std::endl;
    //std::cin.ignore();

    auto node = rclcpp::Node::make_shared("node_ros2_frames_open_palm");

    // 500hz frequency
    rclcpp::WallRate loop_rate(500);

    long motiontime = 0;

    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;

    // rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub =
    //     node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

    // 1 is the queue size
    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);
    const int run_duration_ms = 2000;

    while (rclcpp::ok() && motiontime <= run_duration_ms)
    {
        // since the loop runs at 500hz
        // we add 2, which is miliseconds
        motiontime += 2;

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

        if (motiontime > 0 && motiontime < 1000)
        {
            high_cmd_ros.mode = 1;            // Body control mode
            high_cmd_ros.body_height = 0.0;  // Lower the body to create a sitting posture (adjust as needed)
            high_cmd_ros.euler[0] = 0.0;      // Ensure no forward/backward tilt
            high_cmd_ros.euler[1] = 0.0;      // Add a slight backward tilt for a sitting posture
            high_cmd_ros.euler[2] = 0.0;      // No rotation in ya
        }

        pub->publish(high_cmd_ros);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // Log message for exiting
    RCLCPP_INFO(node->get_logger(), "Completed motion. Node shutting down.");


    rclcpp::shutdown();

    return 0;
}