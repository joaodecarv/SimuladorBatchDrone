#include "SimuladorCore.hpp"

void SimuladorCore::atualizar(Drone& drone, float dt, Commands& cmd) {
    const float velLin = 150.0f;
    const float velAng = 3.0f;

    glm::vec3 frente(sin(drone.yaw), -cos(drone.yaw), 0.0f);
    glm::vec3 direita(cos(drone.yaw), sin(drone.yaw), 0.0f);

    drone.posicao += frente * (cmd.cmdFrente * velLin * dt);
    drone.posicao += direita * (cmd.cmdLado * velLin * dt);
    drone.posicao.z += (cmd.cmdVertical * velLin * dt);
    drone.yaw += (cmd.cmdGiro * velAng * dt);

    drone.yaw = fmodf(drone.yaw, 2.0f * M_PI);
    if (drone.yaw < 0) drone.yaw += 2.0f * M_PI;

    drone.trajetoria.push_back(drone.posicao);

    // Reset para o próximo frame
    cmd = {0, 0, 0, 0};
}

bool SimuladorCore::checarColisao(const Drone& drone, const std::vector<Obstaculo>& obs) {
    for (const auto& o : obs) {
        bool crashX = abs(drone.posicao.x - o.posicao.x) < (drone.comprimento + o.largura) / 2;
        bool crashY = abs(drone.posicao.y - o.posicao.y) < (drone.largura + o.comprimento) / 2;
        bool crashZ = (drone.posicao.z < o.posicao.z + o.altura) && (drone.posicao.z + drone.altura > o.posicao.z);
        
        if (crashX && crashY && crashZ) return true;
    }
    return false;
}