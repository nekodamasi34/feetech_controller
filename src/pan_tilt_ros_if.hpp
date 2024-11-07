#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders; // NOLINT

class PanTiltRosIf : public rclcpp::Node
{
public:
  PanTiltRosIf() : Node("pan_tilt_node")
  {
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("sp1/joy", 10, std::bind(&PanTiltRosIf::onJoyReceived, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(10ms, std::bind(&PanTiltRosIf::onTimer, this));
  }

protected:
  virtual void onTimer() = 0;
  virtual void onJoyReceived(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

  void publishJointState(const sensor_msgs::msg::JointState &msg)
  {
    joint_state_pub_->publish(msg);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
};
