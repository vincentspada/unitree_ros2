#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_state.hpp"
#include "gazebo_msgs/srv/set_model_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"

#include <string>
#include <cmath>
#include <memory>

enum class Coord
{
    WORLD,
    ROBOT
};

class MovePublisherNode : public rclcpp::Node
{
public:
    MovePublisherNode() : Node("move_publisher")
    {
        coord def_frame = Coord::WORLD;

        move_publisher_ = this->create_publisher<gazebo_msgs::msg::ModelState>("/gazebo/set_model_state", 1000);

        gazebo_msgs::msg::ModelState model_state_pub;

        std::string robot_name;
        this->get_parameter("robot_name", robot_name);
        RCLCPP_INFO(this->get_logger(), "robot_name: %s", robot_name.c_str());

        model_state_pub.model_name = robot_name + "_gazebo";
        rclcpp::Rate loop_rate(1000);

        if (def_frame == Coord::WORLD)
        {
            model_state_pub.pose.position.x = 0.0;
            model_state_pub.pose.position.y = 0.0;
            model_state_pub.pose.position.z = 0.5;

            model_state_pub.pose.orientation.x = 0.0;
            model_state_pub.pose.orientation.y = 0.0;
            model_state_pub.pose.orientation.z = 0.0;
            model_state_pub.pose.orientation.w = 1.0;

            model_state_pub.reference_frame = "world";

            long long time_ms = 0;  // time, ms
            const double period = 5000; // ms
            const double radius = 1.5; // m

            while (rclcpp::ok())
            {
                model_state_pub.pose.position.x = radius * std::sin(2 * M_PI * (double)time_ms / period);
                model_state_pub.pose.position.y = radius * std::cos(2 * M_PI * (double)time_ms / period);

                // Convert roll, pitch, yaw to quaternion
                tf2::Quaternion q;
                q.setRPY(0, 0, -2 * M_PI * (double)time_ms / period);
                tf2::convert(q, model_state_pub.pose.orientation);

                move_publisher_->publish(model_state_pub);
                loop_rate.sleep();
                time_ms += 1;
            }
        }
        else if (def_frame == Coord::ROBOT)
        {
            model_state_pub.twist.linear.x = 0.02; // 0.02: 2cm/sec
            model_state_pub.twist.linear.y = 0.0;
            model_state_pub.twist.linear.z = 0.08;

            model_state_pub.twist.angular.x = 0.0;
            model_state_pub.twist.angular.y = 0.0;
            model_state_pub.twist.angular.z = 0.0;

            model_state_pub.reference_frame = "base";

            while (rclcpp::ok())
            {
                move_publisher_->publish(model_state_pub);
                loop_rate.sleep();
            }
        }
    }

private:
    rclcpp::Publisher<gazebo_msgs::msg::ModelState>::SharedPtr move_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
