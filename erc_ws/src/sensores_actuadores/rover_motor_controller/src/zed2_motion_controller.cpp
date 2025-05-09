// MOTORES: XL430-W250
// 0.229 rpm



#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <map> // Para std::map

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/msg/set_velocity.hpp"
#include "custom_interfaces/msg/set_position.hpp" // Asumimos que este mensaje existe

// --- Direcciones de la Tabla de Control para Dynamixel X Series (y compatibles) ---
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_GOAL_POSITION 116   // Dirección para la posición objetivo (4 bytes)
// #define ADDR_PRESENT_VELOCITY 128
// #define ADDR_PRESENT_POSITION 132

// Modos de Operación (valores para ADDR_OPERATING_MODE)
#define MODE_VELOCITY_CONTROL 1
#define MODE_POSITION_CONTROL 3 // Posición estándar. El modo 4 es para multi-vuelta.

// --- Protocolo de Comunicación Dynamixel ---
#define PROTOCOL_VERSION 2.0

// --- Variables Globales para el SDK de Dynamixel ---
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

class Zed2Rotation : public rclcpp::Node
{
public:
    Zed2Rotation()
        : Node("zed2_motion_controller") // Nombre del nodo actualizado
    {
        RCLCPP_INFO(this->get_logger(), "Zed2 Motion Controller node started.");

        this->declare_parameter<std::vector<long int>>("motor_ids", {11L, 12L});
        this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 57600);
        this->declare_parameter<int>("qos_depth", 10);

        std::vector<long int> motor_ids_long;
        this->get_parameter("motor_ids", motor_ids_long);
        for (long int id_long : motor_ids_long)
        {
            if (id_long < 0 || id_long > 253)
            {
                RCLCPP_WARN(this->get_logger(), "Motor ID %ld is out of valid range (0-253). Skipping.", id_long);
                continue;
            }
            uint8_t current_id = static_cast<uint8_t>(id_long);
            this->motor_ids_.push_back(current_id);
            // Asumimos que los motores se inicializan en modo velocidad por la función en main
            this->current_motor_op_modes_[current_id] = MODE_VELOCITY_CONTROL;
        }

        if (this->motor_ids_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid motor IDs configured. Node will not function correctly.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Configured to control motor IDs:");
            for (uint8_t id : this->motor_ids_)
            {
                RCLCPP_INFO(this->get_logger(), "  - %d (Initial mode: Velocity)", id);
            }
        }

        int qos_depth = 0;
        this->get_parameter("qos_depth", qos_depth);
        const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

        set_velocity_subscriber_ = this->create_subscription<custom_interfaces::msg::SetVelocity>(
            "set_velocity",
            QOS_RKL10V,
            std::bind(&Zed2Rotation::handle_set_velocity, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /set_velocity topic.");

        set_position_subscriber_ = this->create_subscription<custom_interfaces::msg::SetPosition>(
            "set_position", // Nuevo topic para control de posición
            QOS_RKL10V,
            std::bind(&Zed2Rotation::handle_set_position, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /set_position topic.");
    }

    ~Zed2Rotation()
    {
        RCLCPP_INFO(this->get_logger(), "Zed2 Motion Controller node stopped.");
    }

    std::vector<uint8_t> motor_ids_;

private:
    std::map<uint8_t, uint8_t> current_motor_op_modes_; // Almacena el modo actual de cada motor

    bool set_torque(uint8_t motor_id, bool enable)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result;
        uint8_t torque_val = enable ? 1 : 0;

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, torque_val, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Failed to %s torque: %s", motor_id, (enable ? "enable" : "disable"), packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        else if (dxl_error != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Motor error while %s torque: %s", motor_id, (enable ? "enable" : "disable"), packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        // RCLCPP_INFO(this->get_logger(), "ID %d: Torque %s.", motor_id, (enable ? "enabled" : "disabled"));
        return true;
    }

    bool ensure_operating_mode(uint8_t motor_id, uint8_t desired_mode)
    {
        if (current_motor_op_modes_.count(motor_id) && current_motor_op_modes_[motor_id] == desired_mode)
        {
            // RCLCPP_DEBUG(this->get_logger(), "ID %d: Already in desired operating mode %d.", motor_id, desired_mode);
            return true; // Ya está en el modo deseado
        }

        RCLCPP_INFO(this->get_logger(), "ID %d: Changing operating mode to %d.", motor_id, desired_mode);
        uint8_t dxl_error = 0;
        int dxl_comm_result;

        // 1. Desactivar Torque
        if (!set_torque(motor_id, false)) return false;

        // 2. Cambiar Modo de Operación
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor_id, ADDR_OPERATING_MODE, desired_mode, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Failed to set operating mode to %d: %s", motor_id, desired_mode, packetHandler->getTxRxResult(dxl_comm_result));
            set_torque(motor_id, true); // Intentar reactivar torque aunque falle el cambio de modo
            return false;
        }
        else if (dxl_error != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Motor error while setting operating mode to %d: %s", motor_id, desired_mode, packetHandler->getRxPacketError(dxl_error));
            set_torque(motor_id, true); // Intentar reactivar torque
            return false;
        }

        // 3. Reactivar Torque
        if (!set_torque(motor_id, true)) return false;

        RCLCPP_INFO(this->get_logger(), "ID %d: Successfully changed operating mode to %d and re-enabled torque.", motor_id, desired_mode);
        current_motor_op_modes_[motor_id] = desired_mode;
        return true;
    }

    void handle_set_velocity(const custom_interfaces::msg::SetVelocity::SharedPtr msg)
    {
        if (std::find(motor_ids_.begin(), motor_ids_.end(), msg->id) == motor_ids_.end())
        {
            return; // No es un motor de nuestra lista
        }

        if (!ensure_operating_mode(msg->id, MODE_VELOCITY_CONTROL))
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Could not set to velocity mode. Aborting command.", msg->id);
            return;
        }

        uint8_t dxl_error = 0;
        int dxl_comm_result;
        uint32_t goal_velocity_value = static_cast<uint32_t>(msg->velocity);

        RCLCPP_DEBUG(this->get_logger(), "ID %d: Setting Goal Velocity to %d (raw: %u)", msg->id, msg->velocity, goal_velocity_value);

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, msg->id, ADDR_GOAL_VELOCITY, goal_velocity_value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Failed to set goal velocity: %s", msg->id, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Motor error while setting goal velocity: %s", msg->id, packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "ID %d: Set Goal Velocity to %d (raw: %u)", msg->id, msg->velocity, goal_velocity_value);
        }
    }

    void handle_set_position(const custom_interfaces::msg::SetPosition::SharedPtr msg)
    {
        if (std::find(motor_ids_.begin(), motor_ids_.end(), msg->id) == motor_ids_.end())
        {
            return; // No es un motor de nuestra lista
        }

        if (!ensure_operating_mode(msg->id, MODE_POSITION_CONTROL))
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Could not set to position mode. Aborting command.", msg->id);
            return;
        }

        uint8_t dxl_error = 0;
        int dxl_comm_result;
        // Asumimos que msg->position es el valor crudo para el motor (e.g., 0-4095).
        // El SDK espera uint32_t para Goal Position.
        // Los valores negativos no tienen sentido para la posición absoluta estándar, pero se hace el cast.
        uint32_t goal_position_value = static_cast<uint32_t>(msg->position);

        // Validar rango de posición si se conoce (ej. 0-4095 para muchos Dynamixels)
        if (msg->position < 0 || msg->position > 4095) { // Ejemplo de rango, ajústalo a tus motores
             RCLCPP_WARN(this->get_logger(), "ID %d: Goal position %d is outside typical range (0-4095). Sending anyway.", msg->id, msg->position);
        }


        RCLCPP_DEBUG(this->get_logger(), "ID %d: Setting Goal Position to %d (raw: %u)", msg->id, msg->position, goal_position_value);

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, msg->id, ADDR_GOAL_POSITION, goal_position_value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Failed to set goal position: %s", msg->id, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Motor error while setting goal position: %s", msg->id, packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "ID %d: Set Goal Position to %d (raw: %u)", msg->id, msg->position, goal_position_value);
        }
    }

    rclcpp::Subscription<custom_interfaces::msg::SetVelocity>::SharedPtr set_velocity_subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::SetPosition>::SharedPtr set_position_subscriber_;
};


// --- Funciones Auxiliares para la Configuración de Dynamixel (usadas desde main) ---

// Esta función ahora configura el motor para un modo operativo inicial y habilita el torque.
// El modo por defecto será Velocidad.
void initialize_motor(dynamixel::PortHandler* ph, dynamixel::PacketHandler* pakh, uint8_t dxl_id, rclcpp::Logger logger) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    uint8_t initial_mode = MODE_VELOCITY_CONTROL; // Modo inicial

    // 1. Desactivar Torque (por si acaso, antes de cambiar modo)
    dxl_comm_result = pakh->write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        RCLCPP_WARN(logger, "ID %d: Could not disable torque before setting initial mode. Error: %s, PacketError: %s. Proceeding...", 
                    dxl_id, pakh->getTxRxResult(dxl_comm_result), pakh->getRxPacketError(dxl_error));
        dxl_error = 0; // Resetear error para el siguiente comando
    }

    // 2. Establecer Modo de Operación Inicial (Velocity Control)
    dxl_comm_result = pakh->write1ByteTxRx(ph, dxl_id, ADDR_OPERATING_MODE, initial_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(logger, "ID %d: Failed to set initial Operating Mode to %d: %s", dxl_id, initial_mode, pakh->getTxRxResult(dxl_comm_result));
        return; 
    } else if (dxl_error != 0) {
        RCLCPP_ERROR(logger, "ID %d: Motor error while setting initial Operating Mode to %d: %s", dxl_id, initial_mode, pakh->getRxPacketError(dxl_error));
        return; 
    }
    RCLCPP_INFO(logger, "ID %d: Initial Operating Mode set to %d (Velocity).", dxl_id, initial_mode);

    // 3. Activar Torque
    dxl_comm_result = pakh->write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(logger, "ID %d: Failed to enable Torque after initial setup: %s", dxl_id, pakh->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_ERROR(logger, "ID %d: Motor error while enabling Torque after initial setup: %s", dxl_id, pakh->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(logger, "ID %d: Torque enabled after initial setup.", dxl_id);
    }
}

void disable_motor_torque_on_shutdown(dynamixel::PortHandler* ph, dynamixel::PacketHandler* pakh, uint8_t dxl_id, rclcpp::Logger logger) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = pakh->write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
     if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN(logger, "ID %d: Failed to disable torque on shutdown: %s", dxl_id, pakh->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_WARN(logger, "ID %d: Motor error while disabling torque on shutdown: %s", dxl_id, pakh->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(logger, "ID %d: Torque disabled for shutdown.", dxl_id);
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Zed2Rotation>();

    bool port_is_open = false;
    std::string device_name_param;
    int baud_rate_param;
    node->get_parameter("device_name", device_name_param);
    node->get_parameter("baud_rate", baud_rate_param);
  
    RCLCPP_INFO(node->get_logger(), "Attempting to use device: %s, baudrate: %d", device_name_param.c_str(), baud_rate_param);

    portHandler = dynamixel::PortHandler::getPortHandler(device_name_param.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler == nullptr || packetHandler == nullptr) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize Dynamixel SDK handlers.");
        rclcpp::shutdown();
        return -1;
    }
  
    if (!portHandler->openPort()) {
      RCLCPP_FATAL(node->get_logger(), "Failed to open port %s!", device_name_param.c_str());
      rclcpp::shutdown();
      return -1;
    }
    port_is_open = true;
    RCLCPP_INFO(node->get_logger(), "Port %s opened.", device_name_param.c_str());
    
    if (!portHandler->setBaudRate(baud_rate_param)) {
      RCLCPP_FATAL(node->get_logger(), "Failed to set baud rate %d!", baud_rate_param);
      if (port_is_open) portHandler->closePort();
      rclcpp::shutdown();
      return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Baud rate set to %d.", baud_rate_param);
    
    if (node->motor_ids_.empty()) {
        RCLCPP_WARN(node->get_logger(), "No motor IDs specified. No motors will be initialized.");
    } else {
        RCLCPP_INFO(node->get_logger(), "Initializing motors...");
        for (uint8_t motor_id : node->motor_ids_) {
          initialize_motor(portHandler, packetHandler, motor_id, node->get_logger());
        }
    }
    
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Shutting down. Disabling torque for configured motors...");
    if (port_is_open) {
      if (!node->motor_ids_.empty()) {
          for (uint8_t motor_id : node->motor_ids_) {
              disable_motor_torque_on_shutdown(portHandler, packetHandler, motor_id, node->get_logger());
          }
      }
      portHandler->closePort();
      RCLCPP_INFO(node->get_logger(), "Port closed.");
    } else {
        RCLCPP_INFO(node->get_logger(), "Port was not open at shutdown.");
    }
  
    rclcpp::shutdown();
    return 0;
}