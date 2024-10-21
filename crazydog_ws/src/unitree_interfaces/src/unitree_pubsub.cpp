#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <unitree_msgs/msg/low_command.hpp>
#include <unitree_msgs/msg/low_state.hpp>
#include <unitree_msgs/msg/motor_command.hpp>
#include <unitree_msgs/msg/motor_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#define WHEEL_RADIUS 0.07  // m

using namespace std::chrono_literals;
using namespace unitree_msgs::msg;


class UnitreeMotor {
public:
    UnitreeMotor(int motor_id = 0, double MAX_degree = 0.0, double MIN_degree = 0.0, double initial_position_check = 0.0) {
        id = motor_id;
        cmd.motorType = MotorType::GO_M8010_6;
        data.motorType = MotorType::GO_M8010_6;

        initial_position_max = initial_position_check + 1.0;
        initial_position_min = initial_position_check - 1.0;
        initial_position = initial_position_check;

        max_position = initial_position_check + MAX_degree;
        min_position = initial_position_check + MIN_degree;
        max = MAX_degree;
        min = MIN_degree;

        cmd.id = motor_id;
    }

    // Public members for easy access.
    int id;
    MotorCmd cmd;
    MotorData data;
    double initial_position_max;
    double initial_position_min;
    double initial_position;
    double max_position;
    double min_position;
    double max;
    double min;
};

class UnitreeCommunication {
public:
    SerialPort serial;
    std::vector<UnitreeMotor> motors;

    UnitreeCommunication(const std::string &device_name) : serial(device_name) {}

    UnitreeMotor* createMotor(int motor_id, double max, double min, double initial_pos) {
        for (auto &motor : motors) {
            if (motor.id == motor_id) return &motor;
        }
        motors.emplace_back(motor_id, max, min, initial_pos);
        return &motors.back();
    }

    void enableAllMotors() {
        for (auto &motor : motors) {
            motor.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
            motor.cmd.q = 0;
            motor.cmd.dq = 0;
            motor.cmd.kp = 0;
            motor.cmd.kd = 0;
            motor.cmd.tau = 0;
            serial.sendRecv(&motor.cmd, &motor.data);
            std::this_thread::sleep_for(10ms);
        }
    }

    void positionForceVelocityCmd(int motor_id, double torque, double kp, double kd, double pos, double vel) {
        for (auto &motor : motors) {
            if (motor.id == motor_id) {
                motor.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                motor.cmd.tau = torque;
                motor.cmd.kp = kp;
                motor.cmd.kd = kd;
                motor.cmd.q = pos;
                motor.cmd.dq = vel * queryGearRatio(MotorType::GO_M8010_6);
            }
        }
    }

    void motorSendRecv() {
        for (auto &motor : motors) {
            if (motor.data.q >= motor.min_position && motor.data.q <= motor.max_position) {
                serial.sendRecv(&motor.cmd, &motor.data);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("unitree_communication"), 
                            "Motor %d out of constraint: [%f, %f]", motor.id, motor.min_position, motor.max_position);
                motor.cmd.q = 0;
                motor.cmd.dq = 0;
                motor.cmd.kp = 0;
                motor.cmd.kd = 0;
                motor.cmd.tau = 0;
                serial.sendRecv(&motor.cmd, &motor.data);
            }
        }
    }
};

class UnitreeInterface : public rclcpp::Node {
private:
    std::shared_ptr<UnitreeCommunication> unitree_left, unitree_right;
    rclcpp::Subscription<LowCommand>::SharedPtr command_sub;
    rclcpp::Publisher<LowState>::SharedPtr status_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr foc_status_sub;
    sensor_msgs::msg::JointState jointstate_msg;
    

public:
    UnitreeInterface() : Node("unitree_pubsub") {
        unitree_left = std::make_shared<UnitreeCommunication>("/dev/unitree-l");
        unitree_right = std::make_shared<UnitreeCommunication>("/dev/unitree-r");
        unitree_left->createMotor(1, 8.475, -5.364, 0.669);
        unitree_left->createMotor(2, 26.801, -1, 1.080);
        unitree_right->createMotor(4, 5.364, -8.475, 7.530); // Adjusted for initial position + 2*pi
        unitree_right->createMotor(5, 1, -26.801, 2.320);

        command_sub = this->create_subscription<LowCommand>(
            "unitree_command", 1, std::bind(&UnitreeInterface::commandCallback, this, std::placeholders::_1));
        status_pub = this->create_publisher<LowState>("unitree_status", 1);
        jointstate_pub = this->create_publisher<sensor_msgs::msg::JointState>("jointstate", 1);
        foc_status_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "foc_msg", 1, std::bind(&UnitreeInterface::focStatusCallback, this, std::placeholders::_1));

        jointstate_msg.name = {"hip_l", "thigh_l", "calf_l", "hip_r", "thigh_r", "calf_r", "wheel_l", "wheel_r"};
        jointstate_msg.position.resize(8, 0.0);
        jointstate_msg.velocity.resize(8, 0.0);

        unitree_left->enableAllMotors();
        unitree_right->enableAllMotors();

        timer_ = this->create_wall_timer(1ms, std::bind(&UnitreeInterface::recvTimerCallback, this));
    }

    void commandCallback(const unitree_msgs::msg::LowCommand::SharedPtr msg) {
        for (size_t id = 0; id < msg->motor_cmd.size(); ++id) {
            int motor_number = static_cast<int>(id);
            double torque = msg->motor_cmd[id].tau;
            double kp = msg->motor_cmd[id].kp;
            double kd = msg->motor_cmd[id].kd;
            double position = msg->motor_cmd[id].q;
            double velocity = msg->motor_cmd[id].dq;

            // Send commands to both sets of motors
            unitree_left->positionForceVelocityCmd(motor_number, torque, kp, kd, position, velocity);
            unitree_right->positionForceVelocityCmd(motor_number, torque, kp, kd, position, velocity);
        }
    }

    void focStatusCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data[0] == 513) {
            jointstate_msg.position[6] = -msg->data[1];
            jointstate_msg.velocity[6] = -msg->data[2] * WHEEL_RADIUS * (2 * M_PI / 60);
        } else if (msg->data[0] == 514) {
            jointstate_msg.position[7] = msg->data[1];
            jointstate_msg.velocity[7] = msg->data[2] * WHEEL_RADIUS * (2 * M_PI / 60);
        }
    }

    void recvTimerCallback() {
        auto t0 = std::chrono::system_clock::now();
        unitree_left->motorSendRecv();
        unitree_right->motorSendRecv();
        auto t1 = std::chrono::system_clock::now();
        auto t0_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(t0.time_since_epoch()).count();
        auto t1_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(t1.time_since_epoch()).count();
        RCLCPP_INFO(this->get_logger(), "%f", t1_seconds-t0_seconds);

        auto msg_list = unitree_msgs::msg::LowState();
        // auto jointstate_msg = sensor_msgs::msg::JointState();

        // Set the header for joint state message
        jointstate_msg.header.stamp = this->get_clock()->now();
        jointstate_msg.header.frame_id = "";
        jointstate_msg.name = {"hip_l", "thigh_l", "calf_l", "hip_r", "thigh_r", "calf_r", "wheel_l", "wheel_r"};

        // Process motors from the first unitree interface
        for (const auto &motor : unitree_left->motors) {
            unitree_msgs::msg::MotorState motor_msg;
            motor_msg.q = static_cast<float>(motor.data.q);
            motor_msg.dq = static_cast<float>(motor.data.dq);
            motor_msg.temperature = static_cast<int>(motor.data.temp);
            int id = motor.id;
            msg_list.motor_state[id] = motor_msg;
            jointstate_msg.position[id] = static_cast<float>(motor.data.q);
            jointstate_msg.velocity[id] = static_cast<float>(motor.data.dq);
        }

        // Process motors from the second unitree interface
        for (const auto &motor : unitree_right->motors) {
            unitree_msgs::msg::MotorState motor_msg;
            motor_msg.q = static_cast<float>(motor.data.q);
            motor_msg.dq = static_cast<float>(motor.data.dq);
            motor_msg.temperature = static_cast<int>(motor.data.temp);

            int id = motor.id;
            msg_list.motor_state[id] = motor_msg;
            jointstate_msg.position[id] = static_cast<float>(motor.data.q);
            jointstate_msg.velocity[id] = static_cast<float>(motor.data.dq);
        }

        // Publish the messages
        status_pub->publish(msg_list);
        jointstate_pub->publish(jointstate_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnitreeInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
