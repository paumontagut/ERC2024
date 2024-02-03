#include <cstdio> // Dynamixel SDK
#include <memory> // Dynamixel SDK
#include <string> // Dynamixel SDK

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/msg/set_velocity.hpp"
#include "custom_interfaces/srv/get_velocity.hpp"
#include "rcutils/cmdline_parser.h" // Dynamixel SDK

#include "motor_controller.hpp"

// Control table for Dynamixel XM540-W270-T/R
#define ADDR_OPERATING_MODE 11 // 1 for velocity control | 3 for position control
#define ADDR_TORQUE_ENABLE 64 // 0 for torque off | 1 for torque on
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default setting
#define BAUDRATE 57600 // 57600 Default baudrate
#define DEVICE_NAME "/dev/ttyUSB0" // ls /dev/ttyUSB* to find the correct device name

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_velocity = 0;
int dxl_comm_result = COMM_TX_FAIL;


MotorController::MotorController()
: Node("motor_controller") {

   RCLCPP_INFO(this->get_logger(), "Motor Controller node started");
   
   this->declare_parameter("qos_depth", 10);
   int8_t qos_depth = 0;
   this->get_parameter("qos_depth", qos_depth);

   const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

   set_velocity_subscriber_ =
      this->create_subscription<SetVelocity>(
      "set_velocity",
      QOS_RKL10V,
      [this](const SetVelocity::SharedPtr msg) -> void {
         uint8_t dxl_error = 0;

         // Velocity value
         uint32_t goal_velocity = (unsigned int)msg->velocity;

         // Write goal velocity (4 bytes)
         dxl_comm_result =
         packetHandler->write4ByteTxRx(
            portHandler, 
            (uint8_t) msg->id, 
            ADDR_GOAL_VELOCITY, 
            goal_velocity, 
            &dxl_error
         );

         if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
         } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
         } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", msg->id, msg->velocity);
         }
      }
   );

   auto get_current_velocity =
      [this](
      const std::shared_ptr<GetVelocity::Request> request,
      std::shared_ptr<GetVelocity::Response> response) -> void {
         
         // Read current velocity (4 bytes)
         dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler, 
            (uint8_t) request->id, 
            ADDR_PRESENT_VELOCITY, 
            reinterpret_cast<uint32_t*>(&current_velocity),
            &dxl_error
         );

         RCLCPP_INFO(
            this->get_logger(),
            "Get [ID: %d] [Current Velocity: %d]",
            request->id,
            current_velocity
         );

         response->velocity = current_velocity;
      };
   get_velocity_server_ = create_service<GetVelocity>("get_velocity", get_current_velocity);
}

MotorController::~MotorController() { RCLCPP_INFO(this->get_logger(), "Motor Controller node stopped"); }

void setupDynamixel(uint8_t dxl_id) {

   // Use Velocity Control mode
   dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, 
      dxl_id, 
      ADDR_OPERATING_MODE, 
      1, 
      &dxl_error
   );

   if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_controller"), "Failed to set Velocity Control mode.");
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_controller"), "Succeeded to set Velocity Control mode.");
   }

   // Enable Torque so the motor can move (EEPROM will be locked)
   dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, 
      dxl_id, 
      ADDR_TORQUE_ENABLE, 
      1, 
      &dxl_error
   );

   if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_controller"), "Failed to enable Torque.");
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_controller"), "Succeeded to enable Torque.");
   }
}

int main(int argc, char * argv[]) {
   portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
   packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
   
   // Open Serial Port
   dxl_comm_result = portHandler->openPort();
   if (dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_controller"), "Failed to open the port!");
      return -1;
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_controller"), "Succeeded to open the port.");
   }
   
   // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
   dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
   if (dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_controller"), "Failed to set the baudrate!");
      return -1;
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_controller"), "Succeeded to set the baudrate.");
   }
   
   setupDynamixel(BROADCAST_ID);
   
   rclcpp::init(argc, argv);
   
   auto motorcontroller = std::make_shared<MotorController>();
   rclcpp::spin(motorcontroller);
   rclcpp::shutdown();
   
   // Disable Torque of DYNAMIXEL
   packetHandler->write1ByteTxRx(
      portHandler,
      BROADCAST_ID,
      ADDR_TORQUE_ENABLE,
      0,
      &dxl_error
   );
   
   return 0;
}