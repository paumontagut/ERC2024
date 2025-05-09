#include <cstdio>    // Para printf, etc. (usado por Dynamixel SDK)
#include <memory>    // Para std::shared_ptr
#include <string>    // Para std::string
#include <vector>    // Para std::vector
#include <algorithm> // Para std::find

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"      // SDK de Dynamixel
#include "custom_interfaces/msg/set_velocity.hpp" // Mensaje para establecer velocidad
// #include "geometry_msgs/msg/twist.hpp" // No necesario para este nodo
// #include "rcutils/cmdline_parser.h" // No es estrictamente necesario si se usan parámetros ROS

// --- Direcciones de la Tabla de Control para Dynamixel X Series (y compatibles) ---
// Consultar el e-manual específico del modelo de motor para confirmar estas direcciones.
#define ADDR_OPERATING_MODE 11   // Dirección para cambiar el modo de operación (1 byte)
                                 // 1: Velocity Control Mode
                                 // 3: Position Control Mode
                                 // 4: Extended Position Control Mode
                                 // 5: Current-based Position Control Mode
                                 // 16: PWM Control Mode
#define ADDR_TORQUE_ENABLE 64    // Dirección para activar/desactivar el torque (1 byte)
                                 // 0: Torque OFF
                                 // 1: Torque ON
#define ADDR_GOAL_VELOCITY 104   // Dirección para escribir la velocidad deseada (4 bytes)
// #define ADDR_PRESENT_VELOCITY 128 // Dirección para leer la velocidad actual (4 bytes) - No usado aquí
// #define ADDR_GOAL_POSITION 116    // Dirección para escribir la posición deseada (4 bytes) - No usado aquí
// #define ADDR_PRESENT_POSITION 132 // Dirección para leer la posición actual (4 bytes) - No usado aquí

// --- Protocolo de Comunicación Dynamixel ---
#define PROTOCOL_VERSION 2.0 // Usualmente 1.0 o 2.0. Los X-Series usan 2.0

// --- Variables Globales para el SDK de Dynamixel ---
// Es común en los ejemplos del SDK que estos sean globales o pasados extensamente.
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

class Zed2Rotation : public rclcpp::Node
{
public:
    Zed2Rotation()
        : Node("zed2_rotation_controller") // Nombre del nodo
    {
        RCLCPP_INFO(this->get_logger(), "Zed2 Rotation Controller node started.");

        // --- Declaración de Parámetros ---
        // Permite configurar el nodo desde un archivo launch o la línea de comandos.
        this->declare_parameter<std::vector<long int>>("motor_ids", {11L, 12L}); // IDs de los motores a controlar
        this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");    // Puerto serial
        this->declare_parameter<int>("baud_rate", 57600);                       // Baudrate para la comunicación
        this->declare_parameter<int>("qos_depth", 10);                          // Profundidad de la cola QoS

        // --- Obtención de Parámetros ---
        std::vector<long int> motor_ids_long;
        this->get_parameter("motor_ids", motor_ids_long);
        // Convertir y validar IDs
        for (long int id_long : motor_ids_long)
        {
            if (id_long < 0 || id_long > 253) // Rango válido para ID en Dynamixel Protocol 2.0 (excepto 254 broadcast)
            {
                RCLCPP_WARN(this->get_logger(), "Motor ID %ld is out of valid range (0-253). Skipping.", id_long);
                continue;
            }
            this->motor_ids_.push_back(static_cast<uint8_t>(id_long));
        }

        if (this->motor_ids_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid motor IDs configured. Node will not function correctly.");
            // Podrías considerar apagar el nodo aquí si es crítico
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Configured to control motor IDs:");
            for (uint8_t id : this->motor_ids_)
            {
                RCLCPP_INFO(this->get_logger(), "  - %d", id);
            }
        }

        // device_name y baud_rate se leen y usan en main() para inicializar portHandler.

        int qos_depth = 0;
        this->get_parameter("qos_depth", qos_depth);
        const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

        // La configuración de los motores (modo, torque) se realiza en main()
        // después de que portHandler y packetHandler estén inicializados.

        // --- Suscriptor para /set_velocity ---
        set_velocity_subscriber_ = this->create_subscription<custom_interfaces::msg::SetVelocity>(
            "set_velocity", // Nombre del topic
            QOS_RKL10V,
            std::bind(&Zed2Rotation::handle_set_velocity, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /set_velocity topic.");
    }

    ~Zed2Rotation()
    {
        RCLCPP_INFO(this->get_logger(), "Zed2 Rotation Controller node stopped.");
        // La desactivación del torque y cierre del puerto se maneja en main(),
        // ya que portHandler y packetHandler son globales y main controla su ciclo de vida.
    }

    // Miembro público para que main pueda acceder a los IDs configurados
    std::vector<uint8_t> motor_ids_;

private:
    void handle_set_velocity(const custom_interfaces::msg::SetVelocity::SharedPtr msg)
    {
        // Verificar si el ID del mensaje es uno de los que este nodo controla
        if (std::find(motor_ids_.begin(), motor_ids_.end(), msg->id) == motor_ids_.end())
        {
            // Este mensaje es para un motor que no gestionamos, lo ignoramos.
            // Se puede activar un RCLCPP_DEBUG si se quiere trazar esto.
            // RCLCPP_DEBUG(this->get_logger(), "Received velocity for ID %d, but not configured to control it. Ignoring.", msg->id);
            return;
        }

        uint8_t dxl_error = 0;
        int dxl_comm_result;

        // El mensaje custom_interfaces/msg/SetVelocity tiene 'id' (uint8) y 'velocity' (int32).
        // La función write4ByteTxRx del SDK espera un uint32_t para el dato.
        // La forma en que los Dynamixel interpretan este valor de velocidad (rango, unidades, signo)
        // depende del modelo específico y su configuración (e.g., Velocity Limit).
        // Por lo general, un int32_t negativo se convierte en un uint32_t grande que el motor
        // interpreta como velocidad en dirección opuesta.
        // 1 unidad de velocidad = 0.229 rpm (para muchos modelos, verificar e-manual).
        uint32_t goal_velocity_value = static_cast<uint32_t>(msg->velocity);

        RCLCPP_DEBUG(this->get_logger(), "Attempting to set ID %d: Goal Velocity (int32): %d, Raw (uint32): %u",
                    msg->id, msg->velocity, goal_velocity_value);

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            msg->id,               // ID del motor específico
            ADDR_GOAL_VELOCITY,    // Dirección del registro para la velocidad objetivo
            goal_velocity_value,   // Valor de velocidad a escribir
            &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Failed to set goal velocity: %s", msg->id,
                         packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ID %d: Error reported by motor while setting goal velocity: %s", msg->id,
                         packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "ID %d: Successfully set Goal Velocity to %d (raw: %u)",
                        msg->id, msg->velocity, goal_velocity_value);
        }
    }

    // Suscriptor al topic de velocidad
    rclcpp::Subscription<custom_interfaces::msg::SetVelocity>::SharedPtr set_velocity_subscriber_;
};


// --- Funciones Auxiliares para la Configuración de Dynamixel (usadas desde main) ---

void configure_motor_for_velocity(dynamixel::PortHandler* ph, dynamixel::PacketHandler* pakh, uint8_t dxl_id, rclcpp::Logger logger) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    // 1. Poner el motor en Modo Control de Velocidad (Operating Mode = 1)
    // Es importante desactivar el Torque (ADDR_TORQUE_ENABLE = 0) ANTES de cambiar el Operating Mode si ya estaba activo.
    // Asumimos que al inicio el torque está desactivado o que el cambio de modo no requiere desactivarlo explícitamente
    // si se hace justo después de energizar el motor. Por seguridad, se podría añadir un disable torque aquí.
    dxl_comm_result = pakh->write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error); // Desactivar torque primero
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        RCLCPP_WARN(logger, "ID %d: Could not disable torque before setting mode. Proceeding anyway... Error: %s, PacketError: %s", 
                    dxl_id, pakh->getTxRxResult(dxl_comm_result), pakh->getRxPacketError(dxl_error));
        // Resetear error para el siguiente comando
        dxl_error = 0; 
    }


    dxl_comm_result = pakh->write1ByteTxRx(ph, dxl_id, ADDR_OPERATING_MODE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(logger, "ID %d: Failed to set Velocity Control mode: %s", dxl_id, pakh->getTxRxResult(dxl_comm_result));
        return; // No continuar si no se puede poner en modo velocidad
    } else if (dxl_error != 0) {
        RCLCPP_ERROR(logger, "ID %d: Motor error while setting Velocity Control mode: %s", dxl_id, pakh->getRxPacketError(dxl_error));
        return; // No continuar
    }
    RCLCPP_INFO(logger, "ID %d: Velocity Control mode set successfully.", dxl_id);

    // 2. Activar el Torque (ADDR_TORQUE_ENABLE = 1)
    // Esto permite que el motor se mueva y mantenga su posición/velocidad.
    // Una vez activado, algunos parámetros (como Operating Mode) no se pueden cambiar sin desactivar el torque.
    dxl_comm_result = pakh->write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(logger, "ID %d: Failed to enable Torque: %s", dxl_id, pakh->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_ERROR(logger, "ID %d: Motor error while enabling Torque: %s", dxl_id, pakh->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(logger, "ID %d: Torque enabled successfully.", dxl_id);
    }
}

void disable_motor_torque(dynamixel::PortHandler* ph, dynamixel::PacketHandler* pakh, uint8_t dxl_id, rclcpp::Logger logger) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = pakh->write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 0, &dxl_error); // 0 para desactivar torque
     if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_WARN(logger, "ID %d: Failed to disable torque on shutdown: %s", dxl_id, pakh->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_WARN(logger, "ID %d: Motor error while disabling torque on shutdown: %s", dxl_id, pakh->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(logger, "ID %d: Torque disabled for shutdown.", dxl_id);
    }
}


// --- Función Principal ---
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Zed2Rotation>(); // Crear el nodo ROS

    // Flag para rastrear si el puerto serial está realmente abierto
    bool port_is_open = false;

    // Obtener parámetros del nodo para la configuración del puerto Dynamixel
    std::string device_name_param;
    int baud_rate_param;
    node->get_parameter("device_name", device_name_param);
    node->get_parameter("baud_rate", baud_rate_param);
  
    RCLCPP_INFO(node->get_logger(), "Attempting to use device: %s", device_name_param.c_str());
    RCLCPP_INFO(node->get_logger(), "Attempting to use baudrate: %d", baud_rate_param);

    // Inicializar PortHandler y PacketHandler (SDK de Dynamixel)
    portHandler = dynamixel::PortHandler::getPortHandler(device_name_param.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler == nullptr || packetHandler == nullptr) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize Dynamixel SDK handlers.");
        rclcpp::shutdown();
        return -1;
    }
  
    // Abrir puerto serial
    if (!portHandler->openPort())
    {
      RCLCPP_FATAL(node->get_logger(), "Failed to open the port: %s! Check permissions (e.g., sudo chmod 666 /dev/ttyUSBX) and if the device is connected.", device_name_param.c_str());
      rclcpp::shutdown(); // Terminar si no se puede abrir el puerto
      return -1;
    }
    port_is_open = true; // Marcar el puerto como abierto
    RCLCPP_INFO(node->get_logger(), "Port %s opened successfully.", device_name_param.c_str());
    
    // Establecer baudrate del puerto
    if (!portHandler->setBaudRate(baud_rate_param))
    {
      RCLCPP_FATAL(node->get_logger(), "Failed to set baud rate to %d! Check if baudrate is supported by the U2D2 or controller.", baud_rate_param);
      if (port_is_open) {
          portHandler->closePort(); // Cerrar puerto antes de salir si estaba abierto
          port_is_open = false;
      }
      rclcpp::shutdown();     // Terminar si no se puede establecer baudrate
      return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Baud rate set to %d successfully.", baud_rate_param);
    
    // Configurar cada motor especificado en los parámetros del nodo
    if (node->motor_ids_.empty()) {
        RCLCPP_WARN(node->get_logger(), "No motor IDs were specified or valid in parameters. No motors will be configured.");
    } else {
        RCLCPP_INFO(node->get_logger(), "Configuring motors...");
        for (uint8_t motor_id : node->motor_ids_)
        {
          configure_motor_for_velocity(portHandler, packetHandler, motor_id, node->get_logger());
        }
    }
    
    rclcpp::spin(node);
    
    // --- Secuencia de Apagado (Shutdown) ---
    RCLCPP_INFO(node->get_logger(), "Shutting down. Disabling torque for configured motors...");
    if (port_is_open) { // Solo intentar operaciones en el puerto si está abierto
      if (node->motor_ids_.empty()) {
          RCLCPP_INFO(node->get_logger(), "No motors were configured, so no torque to disable.");
      } else {
          for (uint8_t motor_id : node->motor_ids_)
          {
              disable_motor_torque(portHandler, packetHandler, motor_id, node->get_logger());
          }
      }
      portHandler->closePort(); // Siempre cerrar el puerto al final
      port_is_open = false;     // Actualizar el flag
      RCLCPP_INFO(node->get_logger(), "Port closed.");
    } else {
        RCLCPP_INFO(node->get_logger(), "Port was not open at shutdown. No torque disabling or port closing attempted.");
    }
  
    rclcpp::shutdown(); // Limpiar recursos de ROS 2
    return 0;
}