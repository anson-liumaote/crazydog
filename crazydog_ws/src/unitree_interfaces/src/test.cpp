#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0),serial("/dev/unitree-l")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        1us, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    SerialPort serial;  // Declare serial here without initialization
    MotorCmd cmd;
    MotorData data;
    void timer_callback()
    {
        cmd.motorType = MotorType::GO_M8010_6;
        data.motorType = MotorType::GO_M8010_6;
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
        cmd.id   = 1;
        cmd.kp   = 0.0;
        cmd.kd   = 0.0;
        cmd.q    = 0.0;
        cmd.dq   = 0.0;
        cmd.tau  = 0.0;
        auto t0 = std::chrono::system_clock::now();
        serial.sendRecv(&cmd,&data);
        auto t1 = std::chrono::system_clock::now();
        auto t0_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(t0.time_since_epoch()).count();
        auto t1_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(t1.time_since_epoch()).count();
        RCLCPP_INFO(this->get_logger(), "%f", t1_seconds-t0_seconds);

        std::cout <<  std::endl;
        std::cout <<  "motor.q: "    << data.q    <<  std::endl;
        std::cout <<  "motor.temp: "   << data.temp   <<  std::endl;
        std::cout <<  "motor.W: "      << data.dq      <<  std::endl;
        std::cout <<  "motor.merror: " << data.merror <<  std::endl;
        std::cout <<  std::endl;

        // usleep(200);

        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
