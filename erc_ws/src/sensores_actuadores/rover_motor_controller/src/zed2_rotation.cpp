#include <cstdio> // Dynamixel SDK
#include <memory> // Dynamixel SDK
#include <string> // Dynamixel SDK

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/srv/get_velocity.hpp"
#include "custom_interfaces/srv/get_position.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcutils/cmdline_parser.h" // Dynamixel SDK
#include "std_msgs/msg/int32.hpp"

// Control table for Dynamixel X Series
#define ADDR_OPERATING_MODE 11 // 1 for velocity control | 3 for position control
#define ADDR_TORQUE_ENABLE 64 // 0 for torque off | 1 for torque on
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default settings
#define BAUDRATE 57600
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0"  // TODO -> ls /dev/ttyUSB* to find the correct device name

// IDs de las herramientas
#define TOOL_HORIZONTAL_ID 5
#define TOOL_VERTICAL_ID 6

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

class Zed2Rotation : public rclcpp::Node {
public:
    Zed2Rotation()
    : Node("zed2_rotation") {

        RCLCPP_INFO(this->get_logger(), "Zed2 Rotation node started");

        this->declare_parameter("qos_depth", 10);
        int8_t qos_depth = 0;
        this->get_parameter("qos_depth", qos_depth);

        const auto QOS_RKL10V = // Defines QoS
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

        setupDynamixel(TOOL_HORIZONTAL_ID);
        setupDynamixel(TOOL_VERTICAL_ID);

        // ╔═════════════════════════════╗
        // ║  SUSCRIPCIÓN GRADOS ZED2    ║
        // ╚═════════════════════════════╝
            
        horizontal_position_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "zed2_rotation_horizontal",
            QOS_RKL10V,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                handleSetPosition(msg->data, TOOL_HORIZONTAL_ID);
            }
        );

        vertical_position_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "zed2_rotation_vertical",
            QOS_RKL10V,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                handleSetPosition(msg->data, TOOL_VERTICAL_ID);
            }
        );
    }

    ~Zed2Rotation() { RCLCPP_INFO(this->get_logger(), "Zed2 Rotation node stopped"); }

private:
    void setupDynamixel(uint8_t dxl_id) {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;

        // Set Position Control Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, 
            dxl_id, 
            ADDR_OPERATING_MODE, 
            3, 
            &dxl_error
        );

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control mode for ID %d.", dxl_id);
        }

        // Enable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, 
            dxl_id, 
            ADDR_TORQUE_ENABLE, 
            1, 
            &dxl_error
        );

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable Torque for ID %d.", dxl_id);
        }
    }

    void handleSetPosition(int goal_position_units, uint8_t dxl_id) {
        uint8_t dxl_error = 0;
        int dxl_comm_result;

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            dxl_id,
            ADDR_GOAL_POSITION,
            goal_position_units,
            &dxl_error
        );

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set position for ID %d.", dxl_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Position: %d units]", 
                dxl_id, goal_position_units);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr horizontal_position_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr vertical_position_subscriber_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    const char* deviceName = DEFAULT_DEVICE_NAME;
    if (argc > 1) {
        deviceName = argv[1]; 
    }
    std::cout << "Using device: " << deviceName << std::endl;

    portHandler = dynamixel::PortHandler::getPortHandler(deviceName);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(rclcpp::get_logger("zed2_rotation"), "Failed to open or configure the port!");
        return -1;
    }

    auto zed2_rotation_node = std::make_shared<Zed2Rotation>();
    rclcpp::spin(zed2_rotation_node);

    rclcpp::shutdown();
    packetHandler->write1ByteTxRx(
        portHandler,
        TOOL_HORIZONTAL_ID,
        ADDR_TORQUE_ENABLE,
        0,
        nullptr
    );
    packetHandler->write1ByteTxRx(
        portHandler,
        TOOL_VERTICAL_ID,
        ADDR_TORQUE_ENABLE,
        0,
        nullptr
    );
    portHandler->closePort();

    return 0;
}
