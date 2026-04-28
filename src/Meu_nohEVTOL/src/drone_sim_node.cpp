/**
 * drone_sim_node.cpp
 *
 * Implementação do nó simulador que substitui o drone PX4 real.
 *
 * Convenção de tópicos PX4 ↔ FSM (via uXRCE-DDS bridge):
 *   FSM   →  PX4 : /fmu/in/...   (aqui: assinado pelo simulador)
 *   PX4   →  FSM : /fmu/out/...  (aqui: publicado pelo simulador)
 *
 * Com namespacing ROS2 (ex: /drone_0), todos os tópicos ficam sob
 * /drone_0/fmu/in/... e /drone_0/fmu/out/...
 */

#include "Meu_nohEVTOL/drone_sim_node.hpp"

#include <cmath>
#include <limits>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace evtol_sim {

// --------------------------------------------------------------------------
// QoS padrão para PX4 (compatível com o uXRCE-DDS bridge)
// --------------------------------------------------------------------------
static rclcpp::QoS px4_qos() {
    return rclcpp::QoS(rclcpp::KeepLast(1))
               .reliability(rclcpp::ReliabilityPolicy::BestEffort)
               .durability(rclcpp::DurabilityPolicy::TransientLocal);
}

// --------------------------------------------------------------------------
// Construtor
// --------------------------------------------------------------------------
DroneSimNode::DroneSimNode(int drone_id, const rclcpp::NodeOptions& options)
    : Node("drone_sim_" + std::to_string(drone_id), options),
      drone_id_(drone_id) {

    RCLCPP_INFO(get_logger(), "DroneSimNode[%d] iniciando...", drone_id_);

    // ------------------------------------------------------------------
    // Publicadores – estado do drone → FSM  (/fmu/out/...)
    // ------------------------------------------------------------------
    pub_local_pos_ = create_publisher<VehicleLocalPosition>(
        "fmu/out/vehicle_local_position", px4_qos());

    pub_status_ = create_publisher<VehicleStatus>(
        "fmu/out/vehicle_status", px4_qos());

    pub_drone_status_ = create_publisher<custom_msgs::msg::DroneStatus>(
        "drone/status", rclcpp::QoS(10));

    // ------------------------------------------------------------------
    // Assinantes – comandos da FSM → drone  (/fmu/in/...)
    // ------------------------------------------------------------------
    sub_traj_ = create_subscription<TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", px4_qos(),
        [this](const TrajectorySetpoint::SharedPtr msg) {
            on_trajectory_setpoint(msg);
        });

    sub_offboard_ = create_subscription<OffboardControlMode>(
        "fmu/in/offboard_control_mode", px4_qos(),
        [this](const OffboardControlMode::SharedPtr msg) {
            on_offboard_control_mode(msg);
        });

    sub_cmd_ = create_subscription<VehicleCommand>(
        "fmu/in/vehicle_command", px4_qos(),
        [this](const VehicleCommand::SharedPtr msg) {
            on_vehicle_command(msg);
        });

    RCLCPP_INFO(get_logger(), "DroneSimNode[%d] pronto.", drone_id_);
}

// --------------------------------------------------------------------------
// reset – volta o drone à posição inicial
// --------------------------------------------------------------------------
void DroneSimNode::reset(double x0, double y0, double z0) {
    state_ = DroneState{};
    state_.x = x0;
    state_.y = y0;
    state_.z = z0;
    last_setpoint_.reset();
    last_offboard_mode_.reset();
    RCLCPP_INFO(get_logger(), "DroneSimNode[%d] resetado para (%.1f, %.1f, %.1f)",
                drone_id_, x0, y0, z0);
}

// --------------------------------------------------------------------------
// trajectory – acesso à trajetória registrada
// --------------------------------------------------------------------------
const std::vector<std::array<double, 3>>& DroneSimNode::trajectory() const {
    return state_.trajectory;
}

// --------------------------------------------------------------------------
// step – avança a simulação dt_s segundos
// --------------------------------------------------------------------------
void DroneSimNode::step(double dt_s) {
    state_.sim_time_us += static_cast<uint64_t>(dt_s * 1e6);

    // Só move o drone se estiver armado e em modo offboard
    if (state_.arming_state == VehicleStatus::ARMING_STATE_ARMED &&
        state_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        update_velocity_from_setpoint(dt_s);
    } else {
        // Desaceleração passiva (atrito do ar)
        const double drag = std::exp(-2.0 * dt_s);
        state_.vx *= drag;
        state_.vy *= drag;
        state_.vz *= drag;
    }

    integrate_position(dt_s);

    // Grava ponto na trajetória se o drone moveu o suficiente
    if (state_.trajectory.empty()) {
        state_.trajectory.push_back({state_.x, state_.y, state_.z});
    } else {
        const auto& last = state_.trajectory.back();
        const double dx = state_.x - last[0];
        const double dy = state_.y - last[1];
        const double dz = state_.z - last[2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) >= kTrajectoryMinStep) {
            state_.trajectory.push_back({state_.x, state_.y, state_.z});
        }
    }

    // Publica estado atualizado para que a FSM possa reagir
    publish_vehicle_local_position();
    publish_vehicle_status();
    publish_drone_status();
}

// --------------------------------------------------------------------------
// Física – atualiza velocidade em direção ao setpoint
// --------------------------------------------------------------------------
void DroneSimNode::update_velocity_from_setpoint(double dt_s) {
    if (!last_setpoint_) return;

    // Posição desejada (NaN = não controla aquele eixo)
    const auto& sp = *last_setpoint_;
    const bool ctrl_x = std::isfinite(sp.position[0]);
    const bool ctrl_y = std::isfinite(sp.position[1]);
    const bool ctrl_z = std::isfinite(sp.position[2]);

    // Velocidade desejada (NaN = não controla)
    const bool vel_x = std::isfinite(sp.velocity[0]);
    const bool vel_y = std::isfinite(sp.velocity[1]);
    const bool vel_z = std::isfinite(sp.velocity[2]);

    // Controle proporcional posição → velocidade desejada
    if (ctrl_x) {
        state_.vx = kPosGain * (static_cast<double>(sp.position[0]) - state_.x);
    } else if (vel_x) {
        state_.vx = static_cast<double>(sp.velocity[0]);
    }

    if (ctrl_y) {
        state_.vy = kPosGain * (static_cast<double>(sp.position[1]) - state_.y);
    } else if (vel_y) {
        state_.vy = static_cast<double>(sp.velocity[1]);
    }

    if (ctrl_z) {
        state_.vz = kPosGain * (static_cast<double>(sp.position[2]) - state_.z);
    } else if (vel_z) {
        state_.vz = static_cast<double>(sp.velocity[2]);
    }

    // Saturação de velocidade
    const double horiz = std::sqrt(state_.vx * state_.vx + state_.vy * state_.vy);
    if (horiz > kMaxHorizVel) {
        const double scale = kMaxHorizVel / horiz;
        state_.vx *= scale;
        state_.vy *= scale;
    }
    state_.vz = clamp(state_.vz, -kMaxVertVel, kMaxVertVel);

    // Yaw (interpolação simples)
    if (std::isfinite(sp.yaw)) {
        double yaw_err = static_cast<double>(sp.yaw) - state_.heading;
        // Normaliza para [-pi, pi]
        while (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
        const double dyaw = clamp(yaw_err, -kHeadingRate * dt_s, kHeadingRate * dt_s);
        state_.heading += dyaw;
    }

    (void)dt_s;
}

// --------------------------------------------------------------------------
// Física – integra posição
// --------------------------------------------------------------------------
void DroneSimNode::integrate_position(double dt_s) {
    state_.x += state_.vx * dt_s;
    state_.y += state_.vy * dt_s;
    state_.z += state_.vz * dt_s;

    // O drone não pode ficar abaixo do chão (z < 0 em NED = acima do solo)
    // z positivo em NED = abaixo do nível de referência.
    // Impedimos que ele atravesse o solo: z <= 0 (não vai para baixo do zero).
    if (state_.z > 0.0 &&
        state_.arming_state != VehicleStatus::ARMING_STATE_ARMED) {
        state_.z = 0.0;
        state_.vz = 0.0;
    }
}

// --------------------------------------------------------------------------
// Clamp auxiliar
// --------------------------------------------------------------------------
double DroneSimNode::clamp(double val, double lo, double hi) {
    return val < lo ? lo : (val > hi ? hi : val);
}

// --------------------------------------------------------------------------
// Callback: TrajectorySetpoint (FSM → simulador)
// --------------------------------------------------------------------------
void DroneSimNode::on_trajectory_setpoint(const TrajectorySetpoint::SharedPtr msg) {
    last_setpoint_ = msg;
}

// --------------------------------------------------------------------------
// Callback: OffboardControlMode (FSM → simulador)
// --------------------------------------------------------------------------
void DroneSimNode::on_offboard_control_mode(const OffboardControlMode::SharedPtr msg) {
    last_offboard_mode_ = msg;

    // Se a FSM está enviando modo offboard e o drone está armado,
    // transicionamos para NAVIGATION_STATE_OFFBOARD
    if (state_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
        state_.nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }
}

// --------------------------------------------------------------------------
// Callback: VehicleCommand (FSM → simulador)
// --------------------------------------------------------------------------
void DroneSimNode::on_vehicle_command(const VehicleCommand::SharedPtr msg) {
    switch (msg->command) {
        // ARM / DISARM
        case VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if (msg->param1 >= 0.5f) {
                state_.arming_state = VehicleStatus::ARMING_STATE_ARMED;
                RCLCPP_INFO(get_logger(), "Drone[%d] ARMADO", drone_id_);
            } else {
                state_.arming_state = VehicleStatus::ARMING_STATE_DISARMED;
                state_.nav_state = VehicleStatus::NAVIGATION_STATE_MANUAL;
                RCLCPP_INFO(get_logger(), "Drone[%d] DESARMADO", drone_id_);
            }
            break;

        // SET MODE (offboard, mission, etc.)
        case VehicleCommand::VEHICLE_CMD_DO_SET_MODE:
            // param1=1 → MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            // Para simplificar: se armado, vai para offboard
            if (state_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
                state_.nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;
            }
            break;

        // NAV_STATE direto (comando PX4 interno)
        case 100001u:  // VEHICLE_CMD_SET_NAV_STATE
            state_.nav_state = static_cast<uint8_t>(msg->param1);
            break;

        // TAKEOFF
        case VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF:
            if (state_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
                state_.nav_state = VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF;
                RCLCPP_INFO(get_logger(), "Drone[%d] DECOLANDO", drone_id_);
            }
            break;

        // LAND
        case VehicleCommand::VEHICLE_CMD_NAV_LAND:
            state_.nav_state = VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
            RCLCPP_INFO(get_logger(), "Drone[%d] POUSANDO", drone_id_);
            break;

        default:
            break;
    }
}

// --------------------------------------------------------------------------
// Publicação: VehicleLocalPosition (simulador → FSM)
// --------------------------------------------------------------------------
void DroneSimNode::publish_vehicle_local_position() {
    VehicleLocalPosition msg;
    msg.timestamp        = state_.sim_time_us;
    msg.timestamp_sample = state_.sim_time_us;

    msg.xy_valid  = true;
    msg.z_valid   = true;
    msg.v_xy_valid = true;
    msg.v_z_valid  = true;

    msg.x = static_cast<float>(state_.x);
    msg.y = static_cast<float>(state_.y);
    msg.z = static_cast<float>(state_.z);

    msg.vx = static_cast<float>(state_.vx);
    msg.vy = static_cast<float>(state_.vy);
    msg.vz = static_cast<float>(state_.vz);

    msg.ax = 0.0f;
    msg.ay = 0.0f;
    msg.az = 0.0f;

    msg.heading = static_cast<float>(state_.heading);
    msg.heading_good_for_control = true;

    // Mantém valores de EPH/EPV baixos (simulação sem ruído)
    msg.eph = 0.01f;
    msg.epv = 0.01f;
    msg.evh = 0.01f;
    msg.evv = 0.01f;

    pub_local_pos_->publish(msg);
}

// --------------------------------------------------------------------------
// Publicação: VehicleStatus (simulador → FSM)
// --------------------------------------------------------------------------
void DroneSimNode::publish_vehicle_status() {
    VehicleStatus msg;
    msg.timestamp = state_.sim_time_us;

    msg.arming_state = state_.arming_state;
    msg.nav_state    = state_.nav_state;
    msg.nav_state_user_intention = state_.nav_state;

    msg.vehicle_type          = VehicleStatus::VEHICLE_TYPE_ROTARY_WING;
    msg.is_vtol               = true;  // EVTOL
    msg.pre_flight_checks_pass = true;
    msg.failsafe              = false;

    pub_status_->publish(msg);
}

// --------------------------------------------------------------------------
// Publicação: DroneStatus customizado (simulador → FSM)
// --------------------------------------------------------------------------
void DroneSimNode::publish_drone_status() {
    custom_msgs::msg::DroneStatus msg;
    msg.header.stamp = now();
    msg.arming_state  = state_.arming_state;
    msg.flight_mode   = state_.nav_state;
    msg.battery_voltage = static_cast<float>(kBatteryVoltage);
    pub_drone_status_->publish(msg);
}

}  // namespace evtol_sim
