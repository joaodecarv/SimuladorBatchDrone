// =============================================================================
// arena.hpp — Tipos de arena para o simulador batch eVTOL
// =============================================================================
// Estas structs espelham as usadas no sae_2025 (fase3/Base, ArenaPoint, etc.)
// São incluídas por drone/Drone.hpp, portanto disponíveis em qualquer estado
// que inclua "drone/Drone.hpp".
// =============================================================================
#pragma once

#include <Eigen/Eigen>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Base de pouso — posição XY + altitude + flag de disponibilidade
// Idêntica à struct Base usada em fase3 do sae_2025.
// ---------------------------------------------------------------------------
struct Base {
    Eigen::Vector3d position{0.0, 0.0, 0.0};  // (x, y, altitude) no frame FRD
    bool active{true};                          // Base ainda disponível para pouso
    std::string id{""};                         // Identificação opcional
};

// ---------------------------------------------------------------------------
// Ponto de waypoint na arena (usado em fase3 para busca de bases)
// ---------------------------------------------------------------------------
struct ArenaPoint {
    Eigen::Vector3d position{0.0, 0.0, 0.0};  // (x, y, altitude) no frame FRD
};

// ---------------------------------------------------------------------------
// Obstáculo estático — caixa AABB
// ---------------------------------------------------------------------------
struct Obstacle {
    Eigen::Vector3d position{0.0, 0.0, 0.0};  // Centro do obstáculo (x, y, alt)
    float width{1.0f};                          // Dimensão em Y (direita)
    float length{1.0f};                         // Dimensão em X (frente)
    float height{2.0f};                         // Dimensão em Z (cima)
    std::string name{""};
};

// ---------------------------------------------------------------------------
// Configuração completa da arena de simulação
// ---------------------------------------------------------------------------
struct Arena {
    float size_x{20.0f};                    // Tamanho em X (Norte/Frente)
    float size_y{20.0f};                    // Tamanho em Y (Leste/Direita)
    std::vector<Obstacle> obstacles;        // Obstáculos estáticos
    std::vector<Base>     bases;            // Bases de pouso disponíveis
    float landing_radius{1.0f};             // Raio para detecção de pouso numa base
};
