
#ifndef __BODY_H__
#define __BODY_H__

#include <rclcpp/rclcpp.hpp>
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"


#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {

extern rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr servo_pub[12];
extern rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr highState_pub;
extern ros2_unitree_legged_msgs::msg::LowCmd lowCmd;
extern ros2_unitree_legged_msgs::msg::LowState lowState;

// extern ros::Publisher servo_pub[12];
// extern ros::Publisher highState_pub;
// extern unitree_legged_msgs::LowCmd lowCmd;
// extern unitree_legged_msgs::LowState lowState;

void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);
}

#endif