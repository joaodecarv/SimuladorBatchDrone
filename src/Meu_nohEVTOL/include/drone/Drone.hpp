// =============================================================================
// drone/Drone.hpp — Drone simulado: DROP-IN para drone_lib/include/drone/Drone.hpp
// =============================================================================
// API pública IDÊNTICA à classe Drone do sae_2025/drone_lib.
// Não usa ROS2 internamente — todo estado é mantido em memória.
// O simulador avança a física chamando drone.sim_step(dt).
//
// SISTEMA DE COORDENADAS (simplificado para simulação):
//   x  = Norte / Frente
//   y  = Leste / Direita
//   alt= altitude (positivo = acima do solo)
//   yaw= azimute (0 = norte, positivo anti-horário visto de cima)
//
// setLocalPosition(x, y, altitude, yaw) : z = altitude (positivo = acima)
// setLocalVelocity(vx, vy, vz, yr)      : vz > 0 = descer (FRD z-down)
// getLocalPosition()                     : retorna (x, y, altitude)
// =============================================================================
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>

// Inclui tipos de arena (Base, ArenaPoint, Obstacle, Arena)
#include "sim/arena.hpp"

// ---------------------------------------------------------------------------
// Namespace DronePX4 — enums de estado (mesmos do drone_lib real)
// ---------------------------------------------------------------------------
namespace DronePX4 {

enum ARMING_STATE { DISARMED = 1, ARMED = 2 };

enum FLIGHT_MODE {
    MANUAL = 0, ALTCTL = 1, POSCTL = 2, AUTO_MISSION = 3,
    AUTO_LOITER = 4, AUTO_RTL = 5, ACRO = 6, DESCEND = 7,
    TERMINATION = 8, OFFBOARD = 9, STAB = 10, AUTO_TAKEOFF = 11,
    AUTO_LAND = 12, AUTO_FOLLOW_TARGET = 13, AUTO_PRECLAND = 14,
    ORBIT = 15, AUTO_VTOL_TAKEOFF = 16, UNKNOWN_MODE = 17
};

enum CONTROLLER_TYPE { NO_CONTROLLER = 0, POSITION = 1, VELOCITY = 2, BODY_RATES = 3 };

enum ARM_DISARM_REASON {
    TRANSITION_TO_STANDBY = 0, RC_STICK = 1, RC_SWITCH = 2,
    COMMAND_INTERNAL = 3, COMMAND_EXTERNAL = 4, MISSION_START = 5,
    SAFETY_BUTTON = 6, AUTO_DISARM_LAND = 7, AUTO_DISARM_PREFLIGHT = 8,
    KILL_SWITCH = 9, LOCKDOWN = 10, FAILURE_DETECTOR = 11,
    SHUTDOWN = 12, ARM_DISARM_REASON_NONE = 13
};

enum FAILURE {
    NONE = 0, ROLL = 1, PITCH = 2, ALT = 4, EXT = 8,
    ARM_ESC = 16, BATTERY = 32, IMBALANCED_PROP = 64, MOTOR = 128
};

struct BoundingBox {
    double center_x{0.5}, center_y{0.5};
    double size_x{0.0},   size_y{0.0};
    double confidence{0.0};
    std::string class_id{"-1"};
};

}  // namespace DronePX4

// ---------------------------------------------------------------------------
// Classe Drone simulada
// ---------------------------------------------------------------------------
class Drone {
public:
    // -----------------------------------------------------------------------
    // Construtor / Destrutor
    // -----------------------------------------------------------------------
    explicit Drone(Eigen::Vector3d init_pos = {0.0, 0.0, 0.0}, float init_yaw = 0.0f)
        : pos_(init_pos), yaw_(init_yaw), initial_yaw_(init_yaw)
    {
        // Posição interna: (x, y, altitude). altitude >= 0.
        sim_start_time_ = std::chrono::steady_clock::now();
    }

    ~Drone() = default;

    // -----------------------------------------------------------------------
    // Getters de estado (API real do Drone)
    // -----------------------------------------------------------------------
    Eigen::Vector3d getLocalPosition() const {
        // Retorna posição relativa ao home, no frame do home
        return pos_ - home_pos_;
    }

    Eigen::Vector3d getOrientation() const {
        return Eigen::Vector3d(0.0, 0.0, static_cast<double>(yaw_ - initial_yaw_));
    }

    float getAltitude() const { return static_cast<float>(pos_.z()); }
    float getGroundSpeed() const { return static_cast<float>(horiz_speed_); }
    float getAirSpeed() const { return 0.0f; }
    double getTime() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - sim_start_time_).count();
    }

    DronePX4::ARMING_STATE getArmingState() const {
        return armed_ ? DronePX4::ARMED : DronePX4::DISARMED;
    }
    DronePX4::FLIGHT_MODE getFlightMode() const {
        if (!offboard_) return DronePX4::MANUAL;
        return DronePX4::OFFBOARD;
    }
    DronePX4::ARM_DISARM_REASON getArmReason()    const { return DronePX4::COMMAND_INTERNAL; }
    DronePX4::ARM_DISARM_REASON getDisarmReason() const { return DronePX4::COMMAND_INTERNAL; }
    DronePX4::FAILURE           getFailure()      const { return DronePX4::NONE; }

    // --- Stubs de sensores (retornam dados vazios no simulador) ---
    // exmeplo de codigo usado anteriormente, mas sem uso nessa versao
    std::vector<DronePX4::BoundingBox> getVerticalBboxes()   const { return {}; }
    std::vector<DronePX4::BoundingBox> getAngledBboxes()     const { return {}; }
    std::vector<DronePX4::BoundingBox> getPostDetections()   const { return {}; }
    std::vector<DronePX4::BoundingBox> getMangueiraDetections() const { return {}; }
    std::vector<DronePX4::BoundingBox> getBlueDetections()   const { return {}; }
    double getMangueiraAngle() const { return 0.0; }
    bool isMangueiraAngleRecent(double = 1.0) const { return false; }
    std::string readQRCode() const { return ""; }
    std::vector<std::string> getHandGestures() const { return {"", ""}; }
    std::array<float, 2> getHandLocation() const { return {0.5f, 0.5f}; }
    void resetHands() {}
    void dropGancho() {}
    void setGroundSpeed(float) {}
    void setAirSpeed(float)   {}
    void create_image_publisher(const std::string&) {}
    void enableCameraSubscriptions(bool = true, bool = true, bool = true) {}
    void disableCameraSubscriptions() {}
    void disableComputerVisionSubscriptions() {}
    void disableCustomMessageSubscriptions() {}

    // -----------------------------------------------------------------------
    // Comandos de controle (API real do Drone)
    // -----------------------------------------------------------------------
    /** Arma imediatamente (versão síncrona: bloqueia até estar armado no drone real) */
    void arm()     { armed_ = true; }
    void armSync() { armed_ = true; }

    /** Desarma imediatamente */
    void disarm()     { armed_ = false; }
    void disarmSync() { armed_ = false; }

    /** Inicia sequência de pouso */
    void land() { landing_ = true; }

    /** Ativa modo offboard imediatamente */
    void toOffboardSync() { offboard_ = true; }
    void toPositionSync() { offboard_ = false; }
    void setOffboardMode() { offboard_ = true; }
    void setOffboardControlMode(DronePX4::CONTROLLER_TYPE) {}

    /** Define posição home para o frame fictício FRD */
    void setHomePosition(const Eigen::Vector3d& /* fictual_home */) {
        // Salva a posição NED atual como origem do frame FRD
        home_pos_     = pos_;
        initial_yaw_  = yaw_;
    }

    /**
     * Envia setpoint de posição em coordenadas FRD relativas ao home.
     *   x   = frente (Norte), y = direita (Leste), z = acima
     *   yaw = azimute (rad), NaN = mantém yaw atual
     */
    void setLocalPosition(float x, float y, float z, float yaw) {
        // Converte FRD relativo ao home → posição absoluta
        pos_setpoint_ = Eigen::Vector3d(
            home_pos_.x() + static_cast<double>(x),
            home_pos_.y() + static_cast<double>(y),
            home_pos_.z() + static_cast<double>(z)
        );
        if (!std::isnan(static_cast<double>(yaw))) {
            yaw_setpoint_ = yaw + initial_yaw_;
        } else {
            yaw_setpoint_ = yaw_;
        }
        pos_control_ = true;
        vel_control_ = false;
    }

    void setLocalPositionSync(double x, double y, double z,
                              double yaw = std::numeric_limits<float>::quiet_NaN(),
                              double /*airspeed*/ = 0.0,
                              double /*dist_threshold*/ = 0.1,
                              DronePX4::CONTROLLER_TYPE = DronePX4::POSITION)
    {
        setLocalPosition(static_cast<float>(x), static_cast<float>(y),
                         static_cast<float>(z), static_cast<float>(yaw));
    }

    /**
     * Envia setpoint de velocidade em frame FRD.
     *   vz > 0 = descer (FRD z-down)
     */
    void setLocalVelocity(float vx, float vy, float vz, float yaw_rate = 0.0f) {
        vel_setpoint_  = Eigen::Vector3d(vx, vy, vz);
        yaw_rate_sp_   = yaw_rate;
        vel_control_   = true;
        pos_control_   = false;
    }

    /** Log — imprime para stdout com prefixo [DRONE-id_] */
    void log(const std::string& info) const {
        std::cout << "[SIM] " << info << "\n";
    }

    // -----------------------------------------------------------------------
    // API exclusiva do simulador (NÃO existe no Drone real)
    // -----------------------------------------------------------------------

    /** Avança a física em dt segundos. Deve ser chamado pelo batch_simulator. */
    void sim_step(double dt) {
        if (!armed_) return;

        Eigen::Vector3d vel(0, 0, 0);
        double yaw_rate = 0.0;

        if (landing_) {
            // Descida simples: 1.5 m/s para baixo
            vel.z() = -1.5;
            pos_control_ = false;
            vel_control_ = false;
        } else if (pos_control_) {
            // Controlador P de posição
            Eigen::Vector3d err = pos_setpoint_ - pos_;
            vel = k_pos_ * err;

            // Satura velocidade
            double horiz = std::hypot(vel.x(), vel.y());
            if (horiz > k_max_horiz_) {
                vel.x() *= k_max_horiz_ / horiz;
                vel.y() *= k_max_horiz_ / horiz;
            }
            vel.z() = std::clamp(vel.z(), -k_max_vert_, k_max_vert_);

            // Yaw: interpolação com taxa limitada
            float yaw_err = yaw_setpoint_ - yaw_;
            // Normaliza para [-π, π]
            while (yaw_err >  static_cast<float>(M_PI)) yaw_err -= 2.0f * static_cast<float>(M_PI);
            while (yaw_err < -static_cast<float>(M_PI)) yaw_err += 2.0f * static_cast<float>(M_PI);
            yaw_rate = std::clamp(static_cast<double>(yaw_err), -k_max_yaw_rate_, k_max_yaw_rate_);

        } else if (vel_control_) {
            // Converte FRD (vx=frente, vy=dir, vz=baixo) para mundo
            // Com yaw=initial_yaw_=0: FRD frente = Norte, direita = Leste, baixo = desc
            double c = std::cos(initial_yaw_), s = std::sin(initial_yaw_);
            vel.x() =  c * vel_setpoint_.x() - s * vel_setpoint_.y();
            vel.y() =  s * vel_setpoint_.x() + c * vel_setpoint_.y();
            vel.z() = -vel_setpoint_.z();  // FRD z-down → altitude-up
            yaw_rate = static_cast<double>(yaw_rate_sp_);
        }

        // Integra posição
        pos_ += vel * dt;
        yaw_ += static_cast<float>(yaw_rate * dt);

        // ── Detecção de colisão ───────────────────────────────────────────
        // Raio do drone para expansão da AABB dos obstáculos (conservador).
        static constexpr double kDroneRadius = 0.3;  // m

        if (!collided_ && arena_ != nullptr) {
            // 1. Paredes laterais da arena (eixo Z livre — só chão/teto não limitados)
            if (pos_.x() < -kDroneRadius ||
                pos_.x() > arena_->size_x + kDroneRadius ||
                pos_.y() < -kDroneRadius ||
                pos_.y() > arena_->size_y + kDroneRadius)
            {
                collided_       = true;
                collision_info_ = "PAREDE";
                armed_          = false;
            }

            // 2. Obstáculos (AABB expandida pelo raio do drone)
            if (!collided_) {
                for (const auto& obs : arena_->obstacles) {
                    // Expande a bounding box pela metade de cada eixo + raio do drone
                    double half_w = obs.width  / 2.0 + kDroneRadius;
                    double half_l = obs.length / 2.0 + kDroneRadius;
                    double top    = obs.height        + kDroneRadius;

                    bool in_x = std::abs(pos_.x() - obs.position.x()) <= half_w;
                    bool in_y = std::abs(pos_.y() - obs.position.y()) <= half_l;
                    bool in_z = pos_.z() <= top;  // obstáculo começa do chão

                    if (in_x && in_y && in_z) {
                        collided_       = true;
                        collision_info_ = "OBSTACULO:" + obs.name;
                        armed_          = false;
                        break;
                    }
                }
            }

            // Se colidiu, reverte posição para antes do passo (freeze)
            if (collided_) {
                pos_ -= vel * dt;
            }
        }
        // ─────────────────────────────────────────────────────────────────

        // Solo: altitude não pode ser negativa
        if (pos_.z() < 0.0) {
            pos_.z() = 0.0;
            if (landing_) {
                landed_ = true;
                armed_  = false;  // Autorematicamente desarma ao pousar
            }
        }

        // Atualiza velocidade horizontal para getGroundSpeed()
        horiz_speed_ = std::hypot(vel.x(), vel.y());

        // Grava trajetória (mínimo 0.05 m entre pontos)
        if (trajectory_.empty() ||
            (pos_ - trajectory_.back()).norm() > traj_min_step_) {
            trajectory_.push_back(pos_);
        }
    }

    /** Define posição absoluta (para inicialização pelo simulador) */
    void sim_set_position(const Eigen::Vector3d& pos, float yaw = 0.0f) {
        pos_         = pos;
        yaw_         = yaw;
        initial_yaw_ = yaw;
        home_pos_    = pos;  // Home = posição inicial
    }

    /**
     * Associa uma arena ao drone para detecção automática de colisões.
     * Deve ser chamado pelo batch_simulator após criar o drone.
     * Verificações por passo:
     *   - Paredes laterais (pos fora de [0, size_x] x [0, size_y])
     *   - Obstáculos (AABB expandida pelo raio do drone, altura incluída)
     * Ao colidir: armed_=false, collided_=true, collision_info_ preenchida.
     */
    void sim_set_arena(const Arena* arena) { arena_ = arena; }

    /** Retorna trajetória absoluta registrada */
    const std::vector<Eigen::Vector3d>& sim_trajectory() const { return trajectory_; }

    /** Posição absoluta atual (não relativa ao home) */
    Eigen::Vector3d sim_pos() const { return pos_; }

    /** True se pousou (altitude atingiu 0 durante landing_) */
    bool sim_has_landed() const { return landed_; }

    /** True se está em processo de aterrissar */
    bool sim_is_landing() const { return landing_; }

    /** True se colidiu com obstáculo ou parede da arena */
    bool sim_has_collided() const { return collided_; }

    /** Descrição do que foi atingido ("PAREDE", "OBSTACULO:nome") */
    const std::string& sim_collision_info() const { return collision_info_; }

    /** Redefine estado para nova tentativa */
    void sim_reset(Eigen::Vector3d init_pos = {0, 0, 0}, float init_yaw = 0.0f) {
        pos_         = init_pos;
        yaw_         = init_yaw;
        initial_yaw_ = init_yaw;
        home_pos_    = init_pos;
        pos_setpoint_ = init_pos;
        yaw_setpoint_ = init_yaw;
        vel_setpoint_ = {0, 0, 0};
        yaw_rate_sp_  = 0.0f;
        pos_control_  = false;
        vel_control_  = false;
        armed_        = false;
        offboard_     = false;
        landing_      = false;
        landed_       = false;
        horiz_speed_    = 0.0;
        collided_       = false;
        collision_info_ = "";
        trajectory_.clear();
    }

private:
    // --- Estado físico ---
    Eigen::Vector3d pos_{0, 0, 0};       // Posição absoluta (x, y, altitude)
    Eigen::Vector3d home_pos_{0, 0, 0};  // Home = origem do frame FRD
    float           yaw_{0.0f};          // Azimute absoluto (rad)
    float           initial_yaw_{0.0f};  // Yaw inicial (gravado pelo setHomePosition)
    double          horiz_speed_{0.0};

    // --- Setpoints ---
    Eigen::Vector3d pos_setpoint_{0, 0, 0};
    float           yaw_setpoint_{0.0f};
    Eigen::Vector3d vel_setpoint_{0, 0, 0};
    float           yaw_rate_sp_{0.0f};

    // --- Flags de modo ---
    bool pos_control_{false};
    bool vel_control_{false};
    bool armed_{false};
    bool offboard_{false};
    bool landing_{false};
    bool landed_{false};
    bool collided_{false};

    // --- Colisão ---
    const Arena* arena_{nullptr};
    std::string  collision_info_{};

    // --- Parâmetros de colisão ---
    // (kDroneRadius definido localmente em sim_step como constexpr)

    // --- Parâmetros físicos ---
    static constexpr double k_pos_          = 2.0;   // Ganho P de posição
    static constexpr double k_max_horiz_    = 5.0;   // Velocidade horiz. máx. (m/s)
    static constexpr double k_max_vert_     = 3.0;   // Velocidade vert. máx. (m/s)
    static constexpr double k_max_yaw_rate_ = 1.0;   // Taxa de yaw máx. (rad/s)
    static constexpr double traj_min_step_  = 0.05;  // Passo mínimo da trajetória (m)

    // --- Trajetória ---
    std::vector<Eigen::Vector3d> trajectory_;

    // --- Tempo de início da simulação ---
    std::chrono::steady_clock::time_point sim_start_time_;
};
