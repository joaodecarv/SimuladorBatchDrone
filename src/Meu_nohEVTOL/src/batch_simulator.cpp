/**
 * batch_simulator.cpp
 *
 * Simulador em lote para FSMs reais da sae_2025.
 *
 * Cada drone usa:
 *   - Drone (include/drone/Drone.hpp)  — drop-in do sae_2025, sem ROS2 interno
 *   - fsm::FSM (include/fsm/fsm.hpp)  — drop-in do sae_2025
 *   - create_fsm() de my_fsm.cpp      — troque esse arquivo pelo seu faseX.cpp
 *
 * Loop de simulação (passo dt, sem tempo real):
 *   1. fsm->execute()     → estado atual age sobre o Drone
 *   2. drone->sim_step(dt) → física avança
 *   3. verifica pouso em base
 *
 * Parâmetros ROS2:
 *   num_drones   (int,    default 4)
 *   sim_steps    (int,    default 1000)
 *   dt           (double, default 0.05)  [s]
 *   output_csv   (string, default "sim_output.csv")
 *   grid_spacing (double, default 5.0)   [m] — espaçamento inicial dos drones
 *
 * Uso:
 *   ros2 run Meu_nohEVTOL batch_simulator \
 *       --ros-args -p num_drones:=4 -p sim_steps:=800 -p dt:=0.05
 */

#include <rclcpp/rclcpp.hpp>

#include "drone/Drone.hpp"  // SimulatedDrone drop-in
#include "fsm/fsm.hpp"      // fsm::FSM drop-in
#include "sim/arena.hpp"    // Arena, Base, Obstacle

#include <Eigen/Dense>

#include <fstream>
#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <cmath>
#include <iomanip>

// ─────────────────────────────────────────────────────────────────────────────
// Declaração antecipada da fábrica implementada em my_fsm.cpp
// (troque my_fsm.cpp pelo seu faseX.cpp preservando esta assinatura)
// ─────────────────────────────────────────────────────────────────────────────
std::unique_ptr<fsm::FSM> create_fsm(Drone* drone);

// ─────────────────────────────────────────────────────────────────────────────
// Arena padrão — edite ou passe sua própria arena via parâmetro
// ─────────────────────────────────────────────────────────────────────────────
static Arena build_default_arena()
{
    Arena arena;
    arena.size_x = 30.0;
    arena.size_y = 30.0;
    arena.landing_radius = 1.5;  // m — raio considerado "pouso na base"

    // Quatro bases dispostas em quadrado
    {Base b; b.id="B0"; b.position=Eigen::Vector3d( 5.0,  5.0, 0.0); b.active=true; arena.bases.push_back(b);}
    {Base b; b.id="B1"; b.position=Eigen::Vector3d(25.0,  5.0, 0.0); b.active=true; arena.bases.push_back(b);}
    {Base b; b.id="B2"; b.position=Eigen::Vector3d( 5.0, 25.0, 0.0); b.active=true; arena.bases.push_back(b);}
    {Base b; b.id="B3"; b.position=Eigen::Vector3d(25.0, 25.0, 0.0); b.active=true; arena.bases.push_back(b);}

    return arena;
}

// ─────────────────────────────────────────────────────────────────────────────
// Estrutura por instância de drone
// ─────────────────────────────────────────────────────────────────────────────
struct DroneInstance {
    int                         id;
    std::unique_ptr<Drone>      drone;
    std::unique_ptr<fsm::FSM>   fsm;
    bool                        landed_on_base = false;
    std::string                 landed_base_id = "";
    bool                        fsm_done       = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Nó ROS2 principal
// ─────────────────────────────────────────────────────────────────────────────
class BatchSimulatorNode : public rclcpp::Node
{
public:
    BatchSimulatorNode()
    : Node("batch_simulator")
    {
        // ── parâmetros ────────────────────────────────────────────────────────
        this->declare_parameter("num_drones",   4);
        this->declare_parameter("sim_steps",    1000);
        this->declare_parameter("dt",           0.05);
        this->declare_parameter("output_csv",   std::string("sim_output.csv"));
        this->declare_parameter("grid_spacing", 5.0);

        int         num_drones   = this->get_parameter("num_drones").as_int();
        int         sim_steps    = this->get_parameter("sim_steps").as_int();
        double      dt           = this->get_parameter("dt").as_double();
        std::string output_csv   = this->get_parameter("output_csv").as_string();
        double      grid_spacing = this->get_parameter("grid_spacing").as_double();

        RCLCPP_INFO(this->get_logger(),
            "BatchSimulator: %d drones, %d steps, dt=%.3f s, csv='%s'",
            num_drones, sim_steps, dt, output_csv.c_str());

        // ── arena ─────────────────────────────────────────────────────────────
        Arena arena = build_default_arena();
        RCLCPP_INFO(this->get_logger(),
            "Arena %.0fx%.0f m, %zu bases, landing_radius=%.1f m",
            arena.size_x, arena.size_y, arena.bases.size(), arena.landing_radius);

        // ── inicialização dos drones ──────────────────────────────────────────
        int cols = std::max(1, static_cast<int>(std::ceil(std::sqrt(num_drones))));
        std::vector<DroneInstance> instances;
        instances.reserve(num_drones);

        for (int i = 0; i < num_drones; ++i) {
            DroneInstance inst;
            inst.id = i;

            // posição inicial em grade
            double x = (i % cols) * grid_spacing;
            double y = (i / cols) * grid_spacing;
            Eigen::Vector3d init_pos(x, y, 0.0);

            inst.drone = std::make_unique<Drone>();
            inst.drone->sim_set_position(init_pos, 0.0f);

            // fabrica a FSM passando o ponteiro do drone
            inst.drone->setHomePosition(init_pos);
            inst.fsm  = create_fsm(inst.drone.get());

            // conecta a arena ao drone (ativa detecção de colisão automática)
            inst.drone->sim_set_arena(&arena);

            instances.push_back(std::move(inst));
            RCLCPP_DEBUG(this->get_logger(),
                "Drone %d inicializado em (%.1f, %.1f, 0.0)", i, x, y);
        }

        // ── CSV header ────────────────────────────────────────────────────────
        std::ofstream csv(output_csv);
        if (!csv.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Não foi possível abrir '%s'", output_csv.c_str());
            return;
        }
        csv << "drone_id,step,sim_time,x,y,altitude,state,landed_on_base,base_id,collided,collision_info\n";

        // ── loop de simulação ─────────────────────────────────────────────────
        auto wall_start = std::chrono::steady_clock::now();
        double sim_time = 0.0;

        for (int step = 0; step < sim_steps; ++step) {
            for (auto& inst : instances) {
                if (inst.fsm_done) continue;

                // 1. FSM age
                try {
                    inst.fsm->execute();
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(),
                        "Drone %d FSM exception no passo %d: %s",
                        inst.id, step, e.what());
                    inst.fsm_done = true;
                    continue;
                }

                // 2. Física avança
                inst.drone->sim_step(dt);

                // 3. Detecta colisão (embutida no sim_step via sim_set_arena)
                if (inst.drone->sim_has_collided() && !inst.fsm_done) {
                    RCLCPP_WARN(this->get_logger(),
                        "Drone %d COLIDIU com %s no passo %d (pos abs: %.2f, %.2f, %.2f)",
                        inst.id,
                        inst.drone->sim_collision_info().c_str(),
                        step,
                        inst.drone->sim_pos().x(),
                        inst.drone->sim_pos().y(),
                        inst.drone->sim_pos().z());
                    inst.fsm_done = true;
                }

                // 4. Detecta FSM terminada
                if (inst.fsm->is_finished()) {
                    inst.fsm_done = true;
                }

                // 5. Detecta pouso em base (somente se drone aterrissou)
                if (!inst.landed_on_base && inst.drone->sim_has_landed()) {
                    Eigen::Vector3d pos = inst.drone->sim_pos();
                    for (const auto& base : arena.bases) {
                        double dist = (pos - base.position).head<2>().norm();
                        if (dist <= arena.landing_radius) {
                            inst.landed_on_base = true;
                            inst.landed_base_id = base.id;
                            RCLCPP_INFO(this->get_logger(),
                                "Drone %d pousou na base '%s' (dist=%.2f m) no passo %d",
                                inst.id, base.id.c_str(), dist, step);
                            break;
                        }
                    }
                    if (!inst.landed_on_base) {
                        RCLCPP_INFO(this->get_logger(),
                            "Drone %d pousou FORA de base no passo %d", inst.id, step);
                    }
                }

                // 6. Escreve linha CSV
                Eigen::Vector3d p = inst.drone->getLocalPosition();
                csv << inst.id << ","
                    << step    << ","
                    << std::fixed << std::setprecision(4) << sim_time << ","
                    << p.x()   << ","
                    << p.y()   << ","
                    << p.z()   << ","
                    << inst.fsm->current_state() << ","
                    << (inst.landed_on_base ? "true" : "false") << ","
                    << inst.landed_base_id << ","
                    << (inst.drone->sim_has_collided() ? "true" : "false") << ","
                    << inst.drone->sim_collision_info()
                    << "\n";
            }
            sim_time += dt;
        }
        csv.close();

        // ── relatório final ───────────────────────────────────────────────────
        auto wall_end = std::chrono::steady_clock::now();
        double wall_s = std::chrono::duration<double>(wall_end - wall_start).count();
        double total_sim_s = sim_steps * dt;
        double speed_ratio = (wall_s > 1e-9) ? (total_sim_s / wall_s) : 0.0;

        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "Simulação concluída:");
        RCLCPP_INFO(this->get_logger(), "  Passos          : %d", sim_steps);
        RCLCPP_INFO(this->get_logger(), "  Tempo simulado  : %.2f s", total_sim_s);
        RCLCPP_INFO(this->get_logger(), "  Tempo real      : %.3f s", wall_s);
        RCLCPP_INFO(this->get_logger(), "  Razão           : %.0fx tempo real", speed_ratio);
        RCLCPP_INFO(this->get_logger(), "  CSV salvo em    : %s", output_csv.c_str());

        int landed_count   = 0;
        int collided_count = 0;
        for (const auto& inst : instances) {
            if (inst.landed_on_base)             ++landed_count;
            if (inst.drone->sim_has_collided())  ++collided_count;
            RCLCPP_INFO(this->get_logger(),
                "  Drone %-2d | FSM: %-12s | Base: %-4s | Colisão: %s",
                inst.id,
                inst.fsm->current_state().c_str(),
                inst.landed_on_base ? inst.landed_base_id.c_str() : "—",
                inst.drone->sim_has_collided() ? inst.drone->sim_collision_info().c_str() : "OK");
        }
        RCLCPP_INFO(this->get_logger(),
            "  Pousaram em base: %d / %d", landed_count, num_drones);
        RCLCPP_INFO(this->get_logger(),
            "  Colisões         : %d / %d", collided_count, num_drones);
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // A simulação roda inteira no construtor — não precisa de spin contínuo
    auto node = std::make_shared<BatchSimulatorNode>();
    rclcpp::shutdown();
    return 0;
}
