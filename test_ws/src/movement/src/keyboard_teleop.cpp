#include <chrono>
#include <cstdio>
#include <stdio.h>
 #include <unistd.h>
 #include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/set_velocity.hpp"

using namespace std::chrono_literals;

#define MIN_ID 1
#define MAX_ID 4
#define MIN_VELOCITY -500
#define MAX_VELOCITY 500

char c;
rclcpp::WallRate loop_rate(100);


int getch(void) {
   int ch;
   struct termios oldt;
   struct termios newt;
 
   // Store old settings, and copy to new settings
   tcgetattr(STDIN_FILENO, &oldt);
   newt = oldt;
 
   // Make required changes and apply the settings
   newt.c_lflag &= ~(ICANON | ECHO);
   newt.c_iflag |= IGNBRK;
   newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
   newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
   newt.c_cc[VMIN] = 1;
   newt.c_cc[VTIME] = 0;
   tcsetattr(fileno(stdin), TCSANOW, &newt);
 
   // Get the current character
   ch = getchar();
 
   // Reapply old settings
   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
 
   return ch;
 }

class KeyboardTeleop : public rclcpp::Node {
public:
  KeyboardTeleop()
  : Node("keyboard_teleop") {
      vel_publisher_ = this->create_publisher<custom_interfaces::msg::SetVelocity>("set_velocity", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&KeyboardTeleop::timer_callback, this));
  }

private:
   void timer_callback() {
      c = getch();
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

int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<KeyboardTeleop>());
   rclcpp::shutdown();
   return 0;
}