#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "body.h"

#include "unitree_model/body.hpp"
#include "unitree_model/odometry.hpp"


using namespace std;
using namespace unitree_model;

bool start_up = true;

class MultiThread : public rclcpp::Node
{
public:
    MultiThread(string rname) : Node("unitree_gazebo_servo")
    {
        robot_name = rname;
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/trunk_imu", 1, std::bind(&MultiThread::imuCallback, this, std::placeholders::_1));
        footForce_sub[0] = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/FR_foot_contact/the_force", 1, std::bind(&MultiThread::FRfootCallback, this, std::placeholders::_1));
        footForce_sub[1] = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/FL_foot_contact/the_force", 1, std::bind(&MultiThread::FLfootCallback, this, std::placeholders::_1));
        footForce_sub[2] = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/RR_foot_contact/the_force", 1, std::bind(&MultiThread::RRfootCallback, this, std::placeholders::_1));
        footForce_sub[3] = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/RL_foot_contact/the_force", 1, std::bind(&MultiThread::RLfootCallback, this, std::placeholders::_1));
        servo_sub[0] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_hip_controller/state", 1, std::bind(&MultiThread::FRhipCallback, this, std::placeholders::_1));
        servo_sub[1] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, std::bind(&MultiThread::FRthighCallback, this, std::placeholders::_1));
        servo_sub[2] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_calf_controller/state", 1, std::bind(&MultiThread::FRcalfCallback, this, std::placeholders::_1));
        servo_sub[3] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_hip_controller/state", 1, std::bind(&MultiThread::FLhipCallback, this, std::placeholders::_1));
        servo_sub[4] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, std::bind(&MultiThread::FLthighCallback, this, std::placeholders::_1));
        servo_sub[5] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_calf_controller/state", 1, std::bind(&MultiThread::FLcalfCallback, this, std::placeholders::_1));
        servo_sub[6] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_hip_controller/state", 1, std::bind(&MultiThread::RRhipCallback, this, std::placeholders::_1));
        servo_sub[7] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, std::bind(&MultiThread::RRthighCallback, this, std::placeholders::_1));
        servo_sub[8] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_calf_controller/state", 1, std::bind(&MultiThread::RRcalfCallback, this, std::placeholders::_1));
        servo_sub[9] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_hip_controller/state", 1, std::bind(&MultiThread::RLhipCallback, this, std::placeholders::_1));
        servo_sub[10] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, std::bind(&MultiThread::RLthighCallback, this, std::placeholders::_1));
        servo_sub[11] = this->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_calf_controller/state", 1, std::bind(&MultiThread::RLcalfCallback, this, std::placeholders::_1));
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        lowState.imu.quaternion[0] = msg->orientation.w;
        lowState.imu.quaternion[1] = msg->orientation.x;
        lowState.imu.quaternion[2] = msg->orientation.y;
        lowState.imu.quaternion[3] = msg->orientation.z;

        lowState.imu.gyroscope[0] = msg->angular_velocity.x;
        lowState.imu.gyroscope[1] = msg->angular_velocity.y;
        lowState.imu.gyroscope[2] = msg->angular_velocity.z;

        lowState.imu.accelerometer[0] = msg->linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg->linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg->linear_acceleration.z;
    }

    void FRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg->mode;
        lowState.motorState[0].q = msg->q;
        lowState.motorState[0].dq = msg->dq;
        lowState.motorState[0].tauEst = msg->tauEst;
    }

    // ... Similar callback functions for other topics ...

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr footForce_sub[4];
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::MotorState>::SharedPtr servo_sub[12];
    string robot_name;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    string robot_name;
    rclcpp::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;
    auto listen_publish_obj = std::make_shared<MultiThread>(robot_name);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(listen_publish_obj);
    std::thread([&executor]() { executor.spin(); }).detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // must wait 300ms to get the first state
    rclcpp::Node n("node_name");
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr lowState_pub =
        n.create_publisher<ros2_unitree_legged_msgs::msg::LowState>(
            "/" + robot_name + "_gazebo/lowState/state", 1);

    // ... Similar code for other publishers ...

    motion_init();

    while (rclcpp::ok())
    {
        /*
        control logic
        */
        lowState_pub->publish(lowState);
        sendServoCmd();

        rclcpp::spin_some(n);
    }

    return 0;
}
