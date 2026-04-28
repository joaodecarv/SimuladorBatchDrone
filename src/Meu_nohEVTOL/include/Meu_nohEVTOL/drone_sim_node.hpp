/**
 * drone_sim_node.hpp
 *
 * Nó ROS2 que simula a controladora PX4 de um drone EVTOL.
 *
 * Este nó substitui o drone real no ambiente de simulação em lote (batching).
 * Ele:
 *   - Assina os comandos que a FSM normalmente enviaria à PX4
 *       /fmu/in/trajectory_setpoint
 *       /fmu/in/offboard_control_mode
 *       /fmu/in/vehicle_command
 *   - Publica o estado do drone que a FSM normalmente leria da PX4
 *       /fmu/out/vehicle_local_position
 *       /fmu/out/vehicle_status
 *   - Implementa física simplificada (integração Euler da posição/velocidade)
 *   - Armazena a trajetória completa para geração de heatmap
 *
 * Ao usar namespaces ROS2 (ex: /drone_0, /drone_1), múltiplas instâncias
 * podem coexistir no mesmo processo sem interferência.
 */

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// PX4 messages
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

// Custom messages
#include "custom_msgs/msg/drone_status.hpp"
#include "custom_msgs/msg/position.hpp"

namespace evtol_sim {

// --------------------------------------------------------------------------
// DroneState – estado interno do modelo físico
// --------------------------------------------------------------------------
struct DroneState {
    // Posição NED (metros)
    double x{0.0}, y{0.0}, z{0.0};
    // Velocidade NED (m/s)
    double vx{0.0}, vy{0.0}, vz{0.0};
    // Orientação (radianos, ângulo yaw em NED)
    double heading{0.0};

    // Status PX4
    uint8_t arming_state{px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED};
    uint8_t nav_state{px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL};

    // Tempo de simulação em microssegundos (avançado manualmente em step())
    uint64_t sim_time_us{0};

    // Histórico de posição para heatmap – armazena [x, y, z] de cada passo
    std::vector<std::array<double, 3>> trajectory;
};

// --------------------------------------------------------------------------
// DroneSimNode
// --------------------------------------------------------------------------
class DroneSimNode : public rclcpp::Node {
public:
    /**
     * @param drone_id   Identificador numérico do drone (usado no nome do nó e
     *                   no namespace se não for fornecido via NodeOptions).
     * @param options    NodeOptions do ROS2, permite injetar namespace externo.
     */
    explicit DroneSimNode(int drone_id,
                          const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // -----------------------------------------------------------------------
    // API do simulador em lote
    // -----------------------------------------------------------------------

    /**
     * Avança a física do drone pelo intervalo dt_s (segundos) e publica o
     * estado atualizado nos tópicos ROS2 para que a FSM possa reagir.
     *
     * Deve ser chamado pelo BatchSimulator a cada iteração do loop de simulação.
     */
    void step(double dt_s);

    /**
     * Retorna a trajetória completa registrada até o momento.
     * Cada entrada é {x, y, z} em NED.
     */
    const std::vector<std::array<double, 3>>& trajectory() const;

    /**
     * Reseta o drone à posição inicial e limpa a trajetória.
     */
    void reset(double x0 = 0.0, double y0 = 0.0, double z0 = 0.0);

    /** Acesso de leitura ao estado interno. */
    const DroneState& state() const { return state_; }

    /** ID numérico do drone. */
    int id() const { return drone_id_; }

private:
    // -----------------------------------------------------------------------
    // Callbacks de assinatura (FSM → simulador)
    // -----------------------------------------------------------------------
    void on_trajectory_setpoint(
        const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);

    void on_offboard_control_mode(
        const px4_msgs::msg::OffboardControlMode::SharedPtr msg);

    void on_vehicle_command(
        const px4_msgs::msg::VehicleCommand::SharedPtr msg);

    // -----------------------------------------------------------------------
    // Publicação do estado (simulador → FSM)
    // -----------------------------------------------------------------------
    void publish_vehicle_local_position();
    void publish_vehicle_status();
    void publish_drone_status();

    // -----------------------------------------------------------------------
    // Física
    // -----------------------------------------------------------------------
    /** Atualiza a velocidade em direção ao setpoint com ganho proporcional. */
    void update_velocity_from_setpoint(double dt_s);
    /** Integra posição com base na velocidade atual. */
    void integrate_position(double dt_s);
    /** Satura uma velocidade ao máximo permitido. */
    static double clamp(double val, double min_val, double max_val);

    // -----------------------------------------------------------------------
    // Campos internos
    // -----------------------------------------------------------------------
    int drone_id_;
    DroneState state_;

    // Último setpoint e modo de controle recebidos da FSM
    px4_msgs::msg::TrajectorySetpoint::SharedPtr last_setpoint_;
    px4_msgs::msg::OffboardControlMode::SharedPtr last_offboard_mode_;

    // Publicadores (simulador → FSM)
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pub_local_pos_;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr pub_status_;
    rclcpp::Publisher<custom_msgs::msg::DroneStatus>::SharedPtr pub_drone_status_;

    // Assinantes (FSM → simulador)
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_traj_;
    rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr sub_offboard_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr sub_cmd_;

    // -----------------------------------------------------------------------
    // Parâmetros do modelo físico (podem ser expostos como parâmetros ROS2)
    // -----------------------------------------------------------------------
    double kPosGain{2.0};         ///< Ganho proporcional posição → velocidade
    double kMaxHorizVel{5.0};     ///< Velocidade horizontal máxima (m/s)
    double kMaxVertVel{3.0};      ///< Velocidade vertical máxima (m/s)
    double kHeadingRate{1.5};     ///< Taxa máxima de rotação do yaw (rad/s)
    double kBatteryVoltage{12.6}; ///< Tensão da bateria simulada (constante)

    // Intervalo mínimo de gravação da trajetória (metros)
    double kTrajectoryMinStep{0.05};
};

}  // namespace evtol_sim
