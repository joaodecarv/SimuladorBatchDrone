/**
 * interfaceFSM.hpp
 *
 * Nó ROS2 que implementa a Máquina de Estados Finitos (FSM) do drone EVTOL.
 *
 * A FSM é a lógica de alto nível que:
 *   - Lê o estado do drone via /fmu/out/vehicle_local_position e /fmu/out/vehicle_status
 *   - Envia comandos via /fmu/in/vehicle_command, /fmu/in/trajectory_setpoint
 *                    e /fmu/in/offboard_control_mode
 *
 * No simulador em lote, esta FSM se comunica com o DroneSimNode que está no
 * mesmo namespace (ex: /drone_0). Isso a torna idêntica ao que rodará no
 * drone real – só muda *com quem* ela troca mensagens.
 *
 * Estados disponíveis:
 *   IDLE      → drone no chão, esperando missão
 *   ARMING    → solicitando armamento (aguardando confirmação do VehicleStatus)
 *   OFFBOARD  → ativando modo offboard (enviando OffboardControlMode continuamente)
 *   TAKEOFF   → subindo até a altitude-alvo
 *   MISSION   → executando waypoints definidos externamente
 *   LANDING   → descendo para pousar
 *   DISARMED  → missão concluída
 */

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "custom_msgs/msg/drone_status.hpp"

namespace evtol_sim {

/** Estados possíveis da FSM */
enum class FsmState : uint8_t {
    IDLE     = 0,
    ARMING   = 1,
    OFFBOARD = 2,
    TAKEOFF  = 3,
    MISSION  = 4,
    LANDING  = 5,
    DISARMED = 6,
};

/** Retorna nome legível do estado */
const char* fsm_state_name(FsmState s);

/**
 * Waypoint simples em NED (metros) + yaw (radianos)
 */
struct Waypoint {
    float x{0.0f}, y{0.0f}, z{0.0f};   // NED
    float yaw{0.0f};                     // radianos
    float acceptance_radius{0.5f};       // metros
};

// --------------------------------------------------------------------------
class InterfaceFSM : public rclcpp::Node {
public:
    /**
     * @param drone_id   ID numérico (utilizado no nome do nó).
     * @param waypoints  Sequência de waypoints da missão.
     * @param options    NodeOptions – deve conter o namespace /drone_N.
     */
    explicit InterfaceFSM(int drone_id,
                          std::vector<Waypoint> waypoints,
                          const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * Executa um ciclo da FSM manualmente (para modo batch).
     * Deve ser chamado DEPOIS de spin_some() do executor, para que os
     * callbacks de assinatura já tenham atualizado o estado interno.
     */
    void tick();

    /** Retorna o estado atual da FSM */
    FsmState current_state() const { return fsm_state_; }

    /** Retorna true quando a missão terminou (DISARMED) */
    bool mission_complete() const { return fsm_state_ == FsmState::DISARMED; }

    /** Reinicia a FSM para uma nova execução */
    void reset(std::vector<Waypoint> new_waypoints = {});

private:
    // Handlers dos estados
    void state_idle();
    void state_arming();
    void state_offboard();
    void state_takeoff();
    void state_mission();
    void state_landing();
    void state_disarmed();

    // Transição entre estados
    void transition_to(FsmState new_state);

    // Callbacks de assinatura
    void on_vehicle_local_position(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void on_vehicle_status(
        const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    // Envio de comandos
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z, float yaw);
    void send_vehicle_command(uint32_t command, float p1 = 0.0f, float p2 = 0.0f);
    void send_arm();
    void send_disarm();

    // Helpers de geometria
    float distance_to(const Waypoint& wp) const;

    // -----------------------------------------------------------------------
    int drone_id_;
    FsmState fsm_state_{FsmState::IDLE};

    // Missão
    std::vector<Waypoint> waypoints_;
    size_t current_wp_{0};
    float target_altitude_{-5.0f};  // NED: negativo = acima do chão

    // Estado do drone (atualizado pelos callbacks)
    px4_msgs::msg::VehicleLocalPosition::SharedPtr local_pos_;
    px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_;

    // Contador de ciclos para evitar spam de comandos
    uint64_t tick_count_{0};
    uint64_t state_entry_tick_{0};

    // Publicadores
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_setpoint_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;

    // Assinantes
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;

    // QoS
    static rclcpp::QoS px4_qos();
};

}  // namespace evtol_sim
