#include <chrono>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/set_velocity.hpp"

using namespace std::chrono_literals;

#define MIN_ID 1
#define MAX_ID 4
#define MIN_VELOCITY -500
#define MAX_VELOCITY 500


class KeyboardTeleop : public rclcpp::Node {
public:
  KeyboardTeleop()
  : Node("keyboard_teleop") {
      vel_publisher_ = this->create_publisher<custom_interfaces::msg::SetVelocity>("set_velocity", 10);
      timer_ = this->create_wall_timer(
        100ms, std::bind(&KeyboardTeleop::timer_callback, this));
  }

private:
   void timer_callback() {
      rclcpp::WallRate loop_rate(100);
      input_ = std::getchar();
      if (input_ == 'w' && (velocity_+50) <= MAX_VELOCITY) {
         velocity_ += 50;
         changes_ = true;
      } else if (input_ == 's' && (velocity_-50) >= MIN_VELOCITY) {
         velocity_ -= 50;
         changes_ = true;
      } else {
         changes_ = false;
      }

      if (changes_) {
         for (int i=1; i<=4; i++) {
            msg_.id = i;
            msg_.velocity = velocity_;
            if (i == 1 || i == 2) { msg_.velocity *= -1; };
            vel_publisher_->publish(msg_);
            loop_rate.sleep();
         }
      }
   }
   rclcpp::Publisher<custom_interfaces::msg::SetVelocity>::SharedPtr vel_publisher_;
   rclcpp::TimerBase::SharedPtr timer_;
   custom_interfaces::msg::SetVelocity msg_;
   int32_t velocity_ = 0;
   char input_;
   bool changes_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}