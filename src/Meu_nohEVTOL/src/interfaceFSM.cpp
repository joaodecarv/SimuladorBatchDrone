/**
 * interfaceFSM.cpp
 *
 * Implementação da Máquina de Estados Finitos (FSM) do drone EVTOL.
 *
 * Ciclo de missão:
 *   IDLE → ARMING → OFFBOARD → TAKEOFF → MISSION → LANDING → DISARMED
 *
 * No simulador em lote, a FSM opera de forma idêntica ao drone real –
 * a única diferença é que os tópicos /fmu/in|out apontam para o
 * DroneSimNode em vez do hardware PX4.
 */

#include "Meu_nohEVTOL/interfaceFSM.hpp"

#include <cmath>
#include <limits>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace evtol_sim {

// --------------------------------------------------------------------------
const char* fsm_state_name(FsmState s) {
    switch (s) {
        case FsmState::IDLE:     return "IDLE";
        case FsmState::ARMING:   return "ARMING";
        case FsmState::OFFBOARD: return "OFFBOARD";
        case FsmState::TAKEOFF:  return "TAKEOFF";
        case FsmState::MISSION:  return "MISSION";
        case FsmState::LANDING:  return "LANDING";
        case FsmState::DISARMED: return "DISARMED";
    }
    return "UNKNOWN";
}

// --------------------------------------------------------------------------
rclcpp::QoS InterfaceFSM::px4_qos() {
    return rclcpp::QoS(rclcpp::KeepLast(1))
               .reliability(rclcpp::ReliabilityPolicy::BestEffort)
               .durability(rclcpp::DurabilityPolicy::TransientLocal);
}

// --------------------------------------------------------------------------
InterfaceFSM::InterfaceFSM(int drone_id,
                             std::vector<Waypoint> waypoints,
                             const rclcpp::NodeOptions& options)
    : Node("fsm_" + std::to_string(drone_id), options),
      drone_id_(drone_id),
      waypoints_(std::move(waypoints)) {

    pub_offboard_ = create_publisher<OffboardControlMode>(
        "fmu/in/offboard_control_mode", px4_qos());
    pub_setpoint_ = create_publisher<TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", px4_qos());
    pub_cmd_ = create_publisher<VehicleCommand>(
        "fmu/in/vehicle_command", px4_qos());

    sub_pos_ = create_subscription<VehicleLocalPosition>(
        "fmu/out/vehicle_local_position", px4_qos(),
        [this](const VehicleLocalPosition::SharedPtr msg) {
            on_vehicle_local_position(msg);
        });
    sub_status_ = create_subscription<VehicleStatus>(
        "fmu/out/vehicle_status", px4_qos(),
        [this](const VehicleStatus::SharedPtr msg) {
            on_vehicle_status(msg);
        });

    RCLCPP_INFO(get_logger(), "InterfaceFSM[%d] pronta – %zu waypoints.",
                drone_id_, waypoints_.size());
}

void InterfaceFSM::reset(std::vector<Waypoint> new_waypoints) {
    if (!new_waypoints.empty()) waypoints_ = std::move(new_waypoints);
    fsm_state_        = FsmState::IDLE;
    current_wp_       = 0;
    tick_count_       = 0;
    state_entry_tick_ = 0;
    local_pos_.reset();
    vehicle_status_.reset();
}

void InterfaceFSM::transition_to(FsmState new_state) {
    RCLCPP_INFO(get_logger(), "Drone[%d] FSM: %s → %s",
                drone_id_, fsm_state_name(fsm_state_), fsm_state_name(new_state));
    fsm_state_        = new_state;
    state_entry_tick_ = tick_count_;
}

void InterfaceFSM::tick() {
    ++tick_count_;

    if (fsm_state_ == FsmState::OFFBOARD ||
        fsm_state_ == FsmState::TAKEOFF  ||
        fsm_state_ == FsmState::MISSION  ||
        fsm_state_ == FsmState::LANDING) {
        publish_offboard_control_mode();
    }

    switch (fsm_state_) {
        case FsmState::IDLE:     state_idle();     break;
        case FsmState::ARMING:   state_arming();   break;
        case FsmState::OFFBOARD: state_offboard(); break;
        case FsmState::TAKEOFF:  state_takeoff();  break;
        case FsmState::MISSION:  state_mission();  break;
        case FsmState::LANDING:  state_landing();  break;
        case FsmState::DISARMED: state_disarmed(); break;
    }
}

// --- Handlers de estado ---

void InterfaceFSM::state_idle() {
    if (tick_count_ >= 5) transition_to(FsmState::ARMING);
}

void InterfaceFSM::state_arming() {
    const uint64_t t = tick_count_ - state_entry_tick_;
    if (t % 10 == 0) send_arm();
    if (vehicle_status_ &&
        vehicle_status_->arming_state == VehicleStatus::ARMING_STATE_ARMED)
        transition_to(FsmState::OFFBOARD);
    if (t > 200) { RCLCPP_WARN(get_logger(), "Drone[%d] Timeout arming!", drone_id_); transition_to(FsmState::IDLE); }
}

void InterfaceFSM::state_offboard() {
    if (local_pos_)
        publish_trajectory_setpoint(local_pos_->x, local_pos_->y, local_pos_->z, local_pos_->heading);
    else
        publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f);

    const uint64_t t = tick_count_ - state_entry_tick_;
    if (t >= 10 && t % 10 == 0)
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);

    if (vehicle_status_ &&
        vehicle_status_->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
        transition_to(FsmState::TAKEOFF);
    if (t > 300) { RCLCPP_WARN(get_logger(), "Drone[%d] Timeout offboard!", drone_id_); transition_to(FsmState::LANDING); }
}

void InterfaceFSM::state_takeoff() {
    const float cz = local_pos_ ? local_pos_->z : 0.0f;
    publish_trajectory_setpoint(
        local_pos_ ? local_pos_->x : 0.0f,
        local_pos_ ? local_pos_->y : 0.0f,
        target_altitude_,
        local_pos_ ? local_pos_->heading : 0.0f);
    if (std::abs(cz - target_altitude_) < 0.5f) {
        RCLCPP_INFO(get_logger(), "Drone[%d] altitude ok (z=%.2f)", drone_id_, cz);
        transition_to(FsmState::MISSION);
    }
}

void InterfaceFSM::state_mission() {
    if (waypoints_.empty() || current_wp_ >= waypoints_.size()) {
        RCLCPP_INFO(get_logger(), "Drone[%d] missão concluída.", drone_id_);
        transition_to(FsmState::LANDING);
        return;
    }
    const Waypoint& wp = waypoints_[current_wp_];
    publish_trajectory_setpoint(wp.x, wp.y, wp.z, wp.yaw);
    if (distance_to(wp) < wp.acceptance_radius) {
        RCLCPP_INFO(get_logger(), "Drone[%d] WP[%zu] atingido.", drone_id_, current_wp_);
        ++current_wp_;
    }
}

void InterfaceFSM::state_landing() {
    const float cx = local_pos_ ? local_pos_->x : 0.0f;
    const float cy = local_pos_ ? local_pos_->y : 0.0f;
    const float cz = local_pos_ ? local_pos_->z : 0.0f;
    publish_trajectory_setpoint(cx, cy, 0.05f, 0.0f);
    const float vz = local_pos_ ? local_pos_->vz : 1.0f;
    if (cz > -0.2f && std::abs(vz) < 0.1f) {
        RCLCPP_INFO(get_logger(), "Drone[%d] pousou.", drone_id_);
        send_disarm();
        transition_to(FsmState::DISARMED);
    }
}

void InterfaceFSM::state_disarmed() {}

// --- Callbacks ---

void InterfaceFSM::on_vehicle_local_position(
    const VehicleLocalPosition::SharedPtr msg) { local_pos_ = msg; }

void InterfaceFSM::on_vehicle_status(
    const VehicleStatus::SharedPtr msg) { vehicle_status_ = msg; }

// --- Publicadores ---

void InterfaceFSM::publish_offboard_control_mode() {
    OffboardControlMode msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.position  = true;
    pub_offboard_->publish(msg);
}

void InterfaceFSM::publish_trajectory_setpoint(
    float x, float y, float z, float yaw) {
    TrajectorySetpoint msg;
    msg.timestamp    = now().nanoseconds() / 1000;
    msg.position     = {x, y, z};
    msg.velocity     = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.yaw          = yaw;
    msg.yawspeed     = NAN;
    pub_setpoint_->publish(msg);
}

void InterfaceFSM::send_vehicle_command(uint32_t command, float p1, float p2) {
    VehicleCommand msg;
    msg.timestamp        = now().nanoseconds() / 1000;
    msg.command          = command;
    msg.param1           = p1;
    msg.param2           = p2;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    pub_cmd_->publish(msg);
}

void InterfaceFSM::send_arm()    { send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f); }
void InterfaceFSM::send_disarm() { send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f); }

float InterfaceFSM::distance_to(const Waypoint& wp) const {
    if (!local_pos_) return std::numeric_limits<float>::max();
    const float dx = local_pos_->x - wp.x;
    const float dy = local_pos_->y - wp.y;
    const float dz = local_pos_->z - wp.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace evtol_sim
