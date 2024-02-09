#include <chrono>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/set_velocity.hpp"

using namespace std::chrono_literals;

#define MIN_ID 1
#define MAX_ID 4
#define MIN_VELOCITY -500
#define MAX_VELOCITY 500

char c;
struct termios orig_termios;


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
      if (c == 'w' && (velocity_+50) <= MAX_VELOCITY) {
         velocity_ += 50;
         changes_ = true;
      } else if (c == 's' && (velocity_-50) >= MIN_VELOCITY) {
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
   bool changes_ = false;
};

void disableRawMode() {
   tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enableRawMode() {
   tcgetattr(STDIN_FILENO, &orig_termios);
   atexit(disableRawMode);

   struct termios raw = orig_termios;
   raw.c_lflag &= ~(ECHO | ICANON);

   tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

int main(int argc, char *argv[]) {
   enableRawMode();
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<KeyboardTeleop>());
   while (read(STDIN_FILENO, &c, 1) == 1);
   rclcpp::shutdown();
   return 0;
}