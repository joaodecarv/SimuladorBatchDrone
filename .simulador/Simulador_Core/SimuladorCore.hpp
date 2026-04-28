#ifndef SIMULADOR_CORE_HPP
#define SIMULADOR_CORE_HPP

#include "../objetos/Drone.hpp"
#include "../objetos/Obstaculo.hpp"
#include <vector>
#include <cmath>

struct Commands {
    float cmdFrente = 0.0f, cmdLado = 0.0f, cmdVertical = 0.0f, cmdGiro = 0.0f;
};

class SimuladorCore {
public:
    void atualizar(Drone& drone, float dt, Commands& cmd);
    bool checarColisao(const Drone& drone, const std::vector<Obstaculo>& obstaculos);
};

#endif