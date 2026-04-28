// =============================================================================
// my_fsm.cpp — ARQUIVO DE TROCA DA FSM
// =============================================================================
//
// ▸ COMO USAR:
//   Substitua este arquivo pelo seu faseX.cpp do sae_2025.
//   São necessárias APENAS 3 adaptações:
//
//   1. Remova     : Drone* drone = new Drone();         (está no construtor da FSM)
//   2. Mude       : this->blackboard_set<Drone>("drone", drone);
//                → this->blackboard_set<Drone>("drone", passed_drone);
//                  (onde passed_drone é o parâmetro do construtor adaptado)
//   3. Adicione   : a função create_fsm() no final do arquivo (veja exemplo abaixo)
//
// ▸ INCLUDES que você PRECISARÁ trocar no seu faseX.cpp:
//   #include "drone/Drone.hpp"   ← já compatível (este simulador fornece)
//   #include "fsm/fsm.hpp"       ← já compatível (este simulador fornece)
//   Remova qualquer #include de mensagens ROS2 usadas só para PX4
//
// ▸ INTERFACE OBRIGATÓRIA (não alterar a assinatura):
//   std::unique_ptr<fsm::FSM> create_fsm(Drone* drone);
//
// =============================================================================
// EXEMPLO ABAIXO: FSM simples de payload — Takeoff → Patrol → Landing
// =============================================================================

#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"
#include "sim/arena.hpp"

#include <Eigen/Eigen>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

// ---------------------------------------------------------------------------
// Estado 1: Decolagem inicial
// ---------------------------------------------------------------------------
class TakeoffState : public fsm::State {
public:
    TakeoffState() = default;

    void on_enter(fsm::Blackboard& blackboard) override {
        drone_           = blackboard.get<Drone>("drone");
        takeoff_alt_     = *blackboard.get<float>("takeoff_height");
        max_vel_         = *blackboard.get<float>("max_vertical_velocity");
        dist_tolerance_  = *blackboard.get<float>("position_tolerance");

        drone_->log("STATE: TAKEOFF");

        // Inicializa offboard e arma
        drone_->toOffboardSync();
        drone_->armSync();
        drone_->setHomePosition(Eigen::Vector3d(0, 0, 0));

        pos_  = drone_->getLocalPosition();
        goal_ = Eigen::Vector3d(pos_.x(), pos_.y(), takeoff_alt_);
    }

    std::string act(fsm::Blackboard& /*blackboard*/) override {
        pos_ = drone_->getLocalPosition();

        if ((pos_ - goal_).norm() < dist_tolerance_) {
            return "TAKEOFF_DONE";
        }

        Eigen::Vector3d diff = goal_ - pos_;
        Eigen::Vector3d step = pos_ + (diff.norm() > max_vel_ ?
                                        diff.normalized() * max_vel_ : diff);
        drone_->setLocalPosition(
            static_cast<float>(step.x()),
            static_cast<float>(step.y()),
            static_cast<float>(step.z()),
            drone_->getOrientation()[2]
        );
        return "";
    }

private:
    Drone*  drone_{nullptr};
    double  takeoff_alt_{5.0};
    double  max_vel_{3.0};
    double  dist_tolerance_{0.3};
    Eigen::Vector3d pos_, goal_;
};

// ---------------------------------------------------------------------------
// Estado 2: Patrol — percorre lista de waypoints
// ---------------------------------------------------------------------------
class PatrolState : public fsm::State {
public:
    PatrolState() = default;

    void on_enter(fsm::Blackboard& blackboard) override {
        drone_      = blackboard.get<Drone>("drone");
        waypoints_  = *blackboard.get<std::vector<ArenaPoint>>("waypoints");
        max_vel_    = *blackboard.get<float>("max_horizontal_velocity");
        tolerance_  = *blackboard.get<float>("position_tolerance");
        wp_idx_     = 0;

        drone_->log("STATE: PATROL — " + std::to_string(waypoints_.size()) + " waypoints");

        if (!waypoints_.empty()) {
            current_goal_ = waypoints_[0].position;
        }
    }

    std::string act(fsm::Blackboard& /*blackboard*/) override {
        if (waypoints_.empty() || wp_idx_ >= static_cast<int>(waypoints_.size())) {
            return "PATROL_DONE";
        }

        Eigen::Vector3d pos = drone_->getLocalPosition();

        if ((pos - current_goal_).norm() < tolerance_) {
            wp_idx_++;
            if (wp_idx_ >= static_cast<int>(waypoints_.size())) {
                return "PATROL_DONE";
            }
            current_goal_ = waypoints_[wp_idx_].position;
            drone_->log("Proximo waypoint: " + std::to_string(wp_idx_));
        }

        Eigen::Vector3d diff = current_goal_ - pos;
        Eigen::Vector3d step = pos + (diff.norm() > max_vel_ ?
                                       diff.normalized() * max_vel_ : diff);
        drone_->setLocalPosition(
            static_cast<float>(step.x()),
            static_cast<float>(step.y()),
            static_cast<float>(step.z()),
            drone_->getOrientation()[2]
        );
        return "";
    }

private:
    Drone*  drone_{nullptr};
    std::vector<ArenaPoint> waypoints_;
    double  max_vel_{5.0};
    double  tolerance_{0.5};
    int     wp_idx_{0};
    Eigen::Vector3d current_goal_{0, 0, 0};
};

// ---------------------------------------------------------------------------
// Estado 3: Pouso — espera 8s (igual ao sae_2025) ou pousa na base mais próxima
// ---------------------------------------------------------------------------
class LandState : public fsm::State {
public:
    LandState() = default;

    void on_enter(fsm::Blackboard& blackboard) override {
        drone_ = blackboard.get<Drone>("drone");
        drone_->log("STATE: LANDING — descendo por 8s");
        start_time_ = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard& /*blackboard*/) override {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (elapsed > 8) {
            return "LANDED";
        }
        // vz > 0 = descer (FRD z-down)
        drone_->setLocalVelocity(0.0f, 0.0f, 0.5f, 0.0f);
        return "";
    }

    void on_exit(fsm::Blackboard& /*blackboard*/) override {
        drone_->land();
        drone_->disarmSync();
        drone_->log("Pousado e desarmado.");
    }

private:
    Drone*  drone_{nullptr};
    std::chrono::steady_clock::time_point start_time_;
};

// ---------------------------------------------------------------------------
// Classe FSM principal — monta a máquina de estados
// ---------------------------------------------------------------------------
class MeuFSM : public fsm::FSM {
public:
    /**
     * Construtor.
     * passed_drone: ponteiro para o Drone simulado criado pelo batch_simulator.
     *
     * ▸ ADAPTE para seu faseX: substitua os valores abaixo pelos seus parâmetros
     *   e adicione/remova estados conforme necessário.
     */
    explicit MeuFSM(Drone* passed_drone)
        : fsm::FSM({"ERROR", "FINISHED"})
    {
        // -- Injeta o drone no blackboard ---------------------------------
        this->blackboard_set<Drone>("drone", passed_drone);

        // -- Parâmetros de voo -------------------------------------------
        this->blackboard_set<float>("takeoff_height",           5.0f);
        this->blackboard_set<float>("max_vertical_velocity",    3.0f);
        this->blackboard_set<float>("max_horizontal_velocity",  5.0f);
        this->blackboard_set<float>("position_tolerance",       0.4f);

        // -- Waypoints de patrulha (quadrado ao redor da origem) ----------
        std::vector<ArenaPoint> waypoints = {
            { Eigen::Vector3d( 5.0,  0.0, 5.0) },
            { Eigen::Vector3d( 5.0,  5.0, 5.0) },
            { Eigen::Vector3d( 0.0,  5.0, 5.0) },
            { Eigen::Vector3d( 0.0,  0.0, 5.0) },
        };
        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);

        // -- Estados -------------------------------------------------------
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("PATROL",  std::make_unique<PatrolState>());
        this->add_state("LAND",    std::make_unique<LandState>());

        // -- Transições ----------------------------------------------------
        this->add_transitions("TAKEOFF", {
            {"TAKEOFF_DONE",  "PATROL"},
            {"SEG FAULT",     "ERROR"}
        });

        this->add_transitions("PATROL", {
            {"PATROL_DONE",   "LAND"},
            {"SEG FAULT",     "ERROR"}
        });

        this->add_transitions("LAND", {
            {"LANDED",        "FINISHED"},
            {"SEG FAULT",     "ERROR"}
        });
    }
};

// ===========================================================================
// FUNÇÃO DE FÁBRICA — NÃO ALTERE A ASSINATURA
// ===========================================================================
/**
 * O batch_simulator chama esta função para criar a FSM de cada drone.
 * Ela recebe o ponteiro para o Drone simulado e deve retornar a FSM configurada.
 *
 * Se você copiar seu faseX.cpp aqui, crie sua subclasse de fsm::FSM acima
 * e retorne-a aqui, passando o drone como argumento.
 */
std::unique_ptr<fsm::FSM> create_fsm(Drone* drone) {
    return std::make_unique<MeuFSM>(drone);
}
