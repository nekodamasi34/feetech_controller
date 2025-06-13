#include "feetech_handler.hpp"
#include "pan_tilt_ros_if.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>

class PanTiltNode : public PanTiltRosIf
{
public:
  PanTiltNode(void) : PanTiltRosIf{}
  {
    printf("start feetech\n");
    std::map<int, ServoConfig> config_list;
    for(int i; i < 255; i++){
        config_list[i] = {-32237, 32236};
    }

    bool open_success = feetech_handler_.Initialize(config_list);
    if (!open_success)
    {
      printf("fail to open serial\n");
      throw;
    }
  }

private:
  /////////////////////////////////////////
  // As req
  void onTimer() override
  {
    feetech_handler_.RequestStatus();

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();

    auto s1_opt = feetech_handler_.GetStatus(21); // 左車輪
    if (s1_opt)
    {
      auto &status = s1_opt.value();
      joint_state.name.push_back("left_pan");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));
    }
    
    auto s2_opt = feetech_handler_.GetStatus(20); // 右車輪
    if (s2_opt)
    {
      auto &status = s2_opt.value();
      joint_state.name.push_back("right_pan");
      joint_state.position.push_back(static_cast<float>(status.position));
      joint_state.velocity.push_back(static_cast<float>(status.velocity));
    }

    publishJointState(joint_state);
  }
  ////////////////////////////////////////////////

  void onJoyReceived(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // joy receive
    float move_speed = msg->axes[1];
    float turn_speed = msg->axes[2];
    float roll_speed = msg->axes[5];
    float unroll_speed = msg->axes[4];

    bool a_button = msg->buttons[0];
    bool b_button = msg->buttons[1];
    bool x_button = msg->buttons[2];
    bool y_button = msg->buttons[3];
    bool up_button = msg->buttons[11];
    bool down_button = msg->buttons[12];

    bool lb_button = msg->buttons[9];
    bool rb_button = msg->buttons[10];

    // 基本的な左右のホイール速度計算
// 基本的な左右のホイール速度計算
float left_wheel_speed = -move_speed + turn_speed;
float right_wheel_speed = move_speed - turn_speed;

// 前進/後退時の旋回動作を調整
if (move_speed != 0.0 && turn_speed != 0.0) {
    // 両方のホイールが動く状態を確保し、片方が停止しないようにする
    if (left_wheel_speed > 1.0) {
        left_wheel_speed = 1.0;
    } else if (left_wheel_speed < -1.0) {
        left_wheel_speed = -1.0;
    }

    if (right_wheel_speed > 1.0) {
        right_wheel_speed = 1.0;
    } else if (right_wheel_speed < -1.0) {
        right_wheel_speed = -1.0;
    }
}

// 前進または後退時の旋回処理
if (turn_speed > 0.0) {
    // 前進時または後退時の右折
    if (move_speed > 0.0) {  // 前進しながら右折
        left_wheel_speed = -move_speed;  // 左ホイールそのまま
        right_wheel_speed = move_speed - turn_speed;  // 右ホイール減速
    } else if (move_speed < 0.0) {  // 後退しながら右折
        left_wheel_speed = -move_speed;  // 左ホイールそのまま
        right_wheel_speed = move_speed + turn_speed;  // 右ホイール減速（後退の場合は符号反転）
    } else if ( move_speed == 0.0 && turn_speed > 0.0) {
      left_wheel_speed = turn_speed;
      right_wheel_speed = 0.0;
    }
} else if (turn_speed < 0.0) {
    // 前進または後退時の左折
    if (move_speed > 0.0) {  // 前進しながら左折
        right_wheel_speed = -move_speed;  // 右ホイールそのまま
        left_wheel_speed = move_speed + turn_speed;  // 左ホイール減速
    } else if (move_speed < 0.0) {  // 後退しながら左折
        right_wheel_speed = -move_speed;  // 右ホイールそのまま
        left_wheel_speed = move_speed - turn_speed;  // 左ホイール減速（後退の場合は符号反転）
    } else if ( move_speed == 0.0 && turn_speed < 0.0) {
      right_wheel_speed = turn_speed;
      left_wheel_speed = 0.0;
    }
}

// 最後にホイール速度の制限
left_wheel_speed = fmax(fmin(left_wheel_speed, 1.0), -1.0);
right_wheel_speed = fmax(fmin(right_wheel_speed, 1.0), -1.0);
    

    RCLCPP_INFO(this->get_logger(), "Left wheel: %.2f, Right wheel: %.2f", left_wheel_speed, right_wheel_speed);

    // サーボモーターに速度を設定
    setCommand(21, left_wheel_speed);  // 左車輪
    setCommand(20, right_wheel_speed); // 右車輪

    // RCLCPP_INFO(this->get_logger(), "Left wheel: %.2f, Right wheel: %.2f", left_wheel_speed, right_wheel_speed);

    // rolled/unrolledの制御
    if ( roll_speed > 0.1 || unroll_speed > 0.1 ) {
      if ( roll_speed > unroll_speed )
    {
      setCommand(22, -roll_speed);
    }
    else if ( roll_speed < unroll_speed )
    {
      setCommand(22, unroll_speed);
    }
    } else {
      setCommand(22, 0);
    }
    

    // 発射の制御
    if (b_button == 1 && x_button == 0)
    {
      setCommandCustomPos(23, 150);
      RCLCPP_INFO(this->get_logger(), "-------------------------lunched-------------------------");
    }
    else if (b_button == 0 && x_button == 1)
    {
      setCommandCustomPos(23, 1050);
      RCLCPP_INFO(this->get_logger(), "-------------------------unlunched-------------------------");
    }

    //lock/unlockの制御
    if (up_button == 1 && down_button == 0)
    {
      setCommandCustomPos(24, 250); // unlock
      RCLCPP_INFO(this->get_logger(), "------------------------unlocked------------------------");
    }
    else if (up_button == 0 && down_button == 1)
    {
      setCommandCustomPos(24, 700); // lock
      RCLCPP_INFO(this->get_logger(), "-------------------------locked-------------------------");
    }

    if (lb_button == 1 && rb_button == 0)
    {
      setTorqueEnable(20, 0);
      setTorqueEnable(21, 0);
      setTorqueEnable(22, 0);
      setTorqueEnable(23, 0);
      setTorqueEnable(24, 0);
      RCLCPP_INFO(this->get_logger(), "-------------------------torque disable-------------------------");
    }
    else if (lb_button == 0 && rb_button == 1)
    {
      setTorqueEnable(20, 1);
      setTorqueEnable(21, 1);
      setTorqueEnable(22, 1);
      setTorqueEnable(23, 1);
      setTorqueEnable(24, 1);
      RCLCPP_INFO(this->get_logger(), "-------------------------torque enable-------------------------");
    }
  }

  /////////////////////////////////////////
  void setCommand(const int id, const float value)
  {
    feetech_handler_.SetCommand(id, 0, static_cast<int>(value * 3150)); // id, position, speed
    // RCLCPP_INFO(this->get_logger(), "%d\n", static_cast<int>(value * 3150));

    usleep(1000); // 1msのガード時間
  }

  void setCommandCustomPos(const int id, const int value)
  {
    feetech_handler_.SetCommand(id, value, 0);
    usleep(100000); // 100ms
  }

  void setTorqueEnable(const int id, const bool state)
  {
    feetech_handler_.SetTorqueEnable(id, state);
    usleep(100000);
  }

private:
  FeetechHandler feetech_handler_;
  static constexpr int center_tick_ = 2048;
  static constexpr float tick_per_rad_ = 651.9f;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto pan_tilt_node = std::make_shared<PanTiltNode>();
  rclcpp::spin(pan_tilt_node);
  rclcpp::shutdown();
  return 0;
}
