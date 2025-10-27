///////////////////////////////////////////////////////////////////////////////
//                          Programa para activar motores
// 
// Se usa de la siguiente manera:
// 
// motor_controller USB ID Modo
//  
// 
//
//
///////////////////////////////////////////////////////////////////////////////

#include <cstdio> // Dynamixel SDK
#include <memory> // Dynamixel SDK
#include <string> // Dynamixel SDK

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/srv/get_velocity.hpp"
#include "custom_interfaces/srv/get_position.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcutils/cmdline_parser.h" // Dynamixel SDK
#include "std_msgs/msg/float32.hpp"


#include "motor_vel_controller.hpp"

// Control table for Dynamixel X Series
#define ADDR_OPERATING_MODE 11 // 1 for velocity control | 3 for position control
#define ADDR_TORQUE_ENABLE 64 // 0 for torque off | 1 for torque on
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default settings
#define BAUDRATE 1000000 // TODO: Es esto verdad?
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0" // ls /dev/ttyUSB* to find the correct device name

// Robot parameters
#define VELOCITY_UNIT 0.229 // TODO: Poner bien -> rpm | See https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/#velocity-limit for more details
#define WHEEL_DIAMETER 0.23 // TODO: Preguntar bien (metros)
#define WHEEL_SEPARATION 0.344 // TODO: Poner bien (metros)

// TODO: Motor IDs
#define RIGHT_FRONT_ID 1
#define RIGHT_REAR_ID 2
#define LEFT_FRONT_ID 3
#define LEFT_REAR_ID 4

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Constants
constexpr double pi = 3.141592653589793;

// Variables for motor control
uint8_t id_herramienta = 3;
uint32_t goal_position = 0;
int32_t right_wheels_velocity = 0;
int32_t left_wheels_velocity = 0;
uint8_t mode = 1; // Velocity mode

// Unit that allows the program to convert desired robot velocity to motor velocity units
const double distance_unit = 1 / (pi * VELOCITY_UNIT * WHEEL_DIAMETER / 60);

// Error handling
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;

void setupDynamixel(uint8_t dxl_id, unit8_t mode) {

        // ----- SET VELOCITY MODE TO ALL MOTORS
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, 
        dxl_id, 
        ADDR_OPERATING_MODE, 
        mode, 
        &dxl_error
    );
   if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_vel_controller"), "Failed to set Velocity Control mode for ID %d.", dxl_id);
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_vel_controller"), "Succeeded to set Velocity Control mode for ID %d.", dxl_id);
   }

   // -------- Enable Torque so the motor can move (EEPROM will be locked)
   // IMPORTANT: Torque must be disabled to change the operating mode
   dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, 
      dxl_id, 
      ADDR_TORQUE_ENABLE, 
      1, 
      &dxl_error
   );

   if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_vel_controller"), "Failed to enable Torque for ID %d.", dxl_id);
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_vel_controller"), "Succeeded to enable Torque for ID %d.", dxl_id);
   }
}

dynamixel::PortHandler init(const char* deviceName, const char* id, uint8_t mode){
      const char* deviceName = DEFAULT_DEVICE_NAME;
      if (id == "Wheels"){
              uint8_t ids[4] = {LEFT_FRONT_ID, LEFT_REAR_ID, RIGHT_FRONT_ID, RIGHT_REAR_ID};
        } else {
            uint8_t ids[1] = std::stoi(id);
        }
   std::cout << "Using device: " << deviceName << std::endl;

   portHandler = dynamixel::PortHandler::getPortHandler(deviceName);
   packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
   
   // Open Serial Port
   dxl_comm_result = portHandler->openPort();
   if (dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_vel_controller"), "Failed to open the port!");
      return -1;
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_vel_controller"), "Succeeded to open the port.");
   }
   
   // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
   dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
   if (dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_vel_controller"), "Failed to set the baudrate!");
      return -1;
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_vel_controller"), "Succeeded to set the baudrate.");
   }
   
   // Initialize Motors with the correct operating mode for each one,
   // and enable Torque for all of them
   for(auto id : ids){
       setupDynamixel(id, mode);
   }
   
   // Keep the node running until closed
   rclcpp::init(argc, argv);
   auto motorcontroller = std::make_shared<MotorController>();
   rclcpp::spin(motorcontroller);
   
   // On shutdown, disable Torque of DYNAMIXEL
   rclcpp::shutdown();

   // Disable Torque of all wheels
    for(auto id : wheel_ids){
        packetHandler->write1ByteTxRx(
            portHandler,
            id,
            ADDR_TORQUE_ENABLE,
            0,
            &dxl_error
        );
    }
    return portHandler;
}

int main(int argc, char * argv[]) {
   
   // Get custom device name
   if (argc > 1) {
      deviceName = argv[1]; 
      if (argc > 2){
         ids = argv[2];
        if (argc > 3){
            if (argv[3] == "Velocity"){
                mode = 1;
            } else if (argv[3] == "Position"){
                mode = 3;
            }
        }
    }
   }
   init(deviceName, ids, mode);
}

   return 0;
}
