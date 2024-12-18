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
                // ╔══════════════════════════════════════════════════════════════════════════════╗
                // ║ Conversión: rango de -90° (izquierda) a 90° (derecha)                        ║
                // ║ 0° -> Frente (centro) | -90° -> 0 unidades | 90° -> 2047 unidades           ║
                // ║                                                                  [2047 u]   ║
                // ║                               |---->  90° derecha    → Máx [2047 u]         ║
                // ║                               |---->  0°  frente     → Medio [1024 u]       ║
                // ║                               |----> -90° izquierda  → Mín [0 u]           ║
                // ╚══════════════════════════════════════════════════════════════════════════════╝
                
                int angle_degrees = msg->data;

                // Limitar la entrada entre -90 y 90 grados
                if (angle_degrees < -90) angle_degrees = -90;
                if (angle_degrees > 90) angle_degrees = 90;

                // Convertir el rango [-90, 90] a [0, 2047]
                int goal_position_units = (int)(((float)(angle_degrees + 90) / 180.0f) * 2047.0f);

                RCLCPP_INFO(this->get_logger(),
                            "Recibidos %d grados (horizontal), convertidos a %d unidades",
                            angle_degrees, goal_position_units);

                handleSetPosition(goal_position_units, TOOL_HORIZONTAL_ID);
            }
        );


        vertical_position_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            // ID: 6
            "zed2_rotation_vertical",
            QOS_RKL10V,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                int angle_degrees = msg->data;
                if (angle_degrees < 0) angle_degrees = 0;
                if (angle_degrees > 360) angle_degrees = 360;
                int goal_position_units = (int)((float)angle_degrees * (4095.0f / 360.0f));
                RCLCPP_INFO(this->get_logger(), "Recibidos %d grados (vertical), convertidos a %d unidades",
                    angle_degrees, goal_position_units);
                handleSetPosition(goal_position_units, TOOL_VERTICAL_ID);
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
            RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control mode for ID %d: %s", dxl_id,
                         packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            if (dxl_error != 0) {
                RCLCPP_WARN(this->get_logger(), "Position Control mode set with warning for ID %d: %s",
                            dxl_id, packetHandler->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Position Control mode set successfully for ID %d", dxl_id);
            }
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
            RCLCPP_ERROR(this->get_logger(), "Failed to enable Torque for ID %d: %s", dxl_id,
                         packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            if (dxl_error != 0) {
                RCLCPP_WARN(this->get_logger(), "Torque enabled with warning for ID %d: %s",
                            dxl_id, packetHandler->getRxPacketError(dxl_error));
            } else {
                RCLCPP_INFO(this->get_logger(), "Torque enabled successfully for ID %d", dxl_id);
            }
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

    if (!portHandler->openPort()) {
        RCLCPP_ERROR(rclcpp::get_logger("zed2_rotation"), "Failed to open the port!");
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("zed2_rotation"), "Port opened successfully.");
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(rclcpp::get_logger("zed2_rotation"), "Failed to set baud rate!");
        portHandler->closePort();
        return -1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("zed2_rotation"), "Baud rate set to %d.", BAUDRATE);
    }

    auto zed2_rotation_node = std::make_shared<Zed2Rotation>();
    rclcpp::spin(zed2_rotation_node);

    rclcpp::shutdown();

    uint8_t dxl_error = 0;
    int dxl_comm_result;

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        TOOL_HORIZONTAL_ID,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("zed2_rotation"), "Failed to disable torque for ID %d: %s", TOOL_HORIZONTAL_ID,
                     packetHandler->getTxRxResult(dxl_comm_result));
    } else {
        if (dxl_error != 0) {
            RCLCPP_WARN(rclcpp::get_logger("zed2_rotation"), "Torque disabled with warning for ID %d: %s",
                        TOOL_HORIZONTAL_ID, packetHandler->getRxPacketError(dxl_error));
        } else {
            RCLCPP_INFO(rclcpp::get_logger("zed2_rotation"), "Torque disabled for ID %d", TOOL_HORIZONTAL_ID);
        }
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        TOOL_VERTICAL_ID,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("zed2_rotation"), "Failed to disable torque for ID %d: %s", TOOL_VERTICAL_ID,
                     packetHandler->getTxRxResult(dxl_comm_result));
    } else {
        if (dxl_error != 0) {
            RCLCPP_WARN(rclcpp::get_logger("zed2_rotation"), "Torque disabled with warning for ID %d: %s",
                        TOOL_VERTICAL_ID, packetHandler->getRxPacketError(dxl_error));
        } else {
            RCLCPP_INFO(rclcpp::get_logger("zed2_rotation"), "Torque disabled for ID %d", TOOL_VERTICAL_ID);
        }
    }

    portHandler->closePort();
    RCLCPP_INFO(rclcpp::get_logger("zed2_rotation"), "Port closed. Exiting...");

    return 0;
}