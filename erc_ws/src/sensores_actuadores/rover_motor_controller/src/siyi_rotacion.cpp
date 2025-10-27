#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "motor_controller.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sdk.h" //Mirar como importarlo bien

uint8_t mode = 3; // Position
uint8_t dxl_id = 14; // Cambiar al que toca
uint8_t dxl_error = 0;
int dxl_comm_result;
const char* deviceName;
const char* id = 14; //Revisar id
uint8_t mode = 3; //Posición
dynamixel::PortHandler portHandler;

#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0" 
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_OPERATING_MODE 11 // 1 for velocity control | 3 for position control

void move_yaw(const std_msgs::msg::Int32::SharedPtr msg)
{
  int angle_degrees = msg->data;

    // Limitar la entrada entre -90 y 90 grados ¿¿¿¿¿¿ Es necesario ???????
    if (angle_degrees < -90) angle_degrees = -90;
    if (angle_degrees > 90) angle_degrees = 90;

    // Convertir el rango [-90, 90] a [0, 2047]
    int goal_position_units = (int)(((float)(angle_degrees + 90) / 180.0f) * 2047.0f);
    RCLCPP_INFO(this->get_logger(),
                            "Recibidos %d grados (horizontal), convertidos a %d unidades",
                            angle_degrees, goal_position_units);

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            dxl_id,
            ADDR_GOAL_POSITION,
            goal_position_units,
            &dxl_error
        );

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set position for ID %d: %s", dxl_id,
                         packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            if (dxl_error != 0) {
                RCLCPP_WARN(this->get_logger(), "Position set with warning for ID %d: %s",
                            dxl_id, packetHandler->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Position: %d units]", 
                    dxl_id, goal_position_units);
            }
        }
}

void move_pitch(const std_msgs::msg::Int32::SharedPtr msg){
    int angle = msg->data;
    if (angle < -90) angle = -90;
    if (angle > 25) angle = 25;
    sdk.set_gimbal_angles(0,angle);
}


int main(int argc, char * argv[])
{
    if (argc > 1){
        deviceName = argv[1]; 
    } else {
        deviceName = DEFAULT_DEVICE_NAME;
    }
  portHandler = motor_controller::init(deviceName, id, mode);
  rclcpp::init(argc, argv);
  SIYI_SDK sdk("192.168.144.25");
  auto node = rclcpp::Node::make_shared("Siyi_rotation");
  auto subscription =
    node ->create_subscription<std_msgs::msg::Int32>("/gui/siyi_rotation/horizontal", 10, move_yaw);
  auto subscription =
    node ->create_subscription<std_msgs::msg::Int32>("/gui/siyi_rotation/vertical", 10, move_pitch);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
