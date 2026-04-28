#ifndef DRONE_HPP
#define DRONE_HPP

#include <glm/glm.hpp>
#include <vector>

struct Drone {
    glm::vec3 posicao = {400.0f, 300.0f, 10.0f};
    float yaw = 0.0f;
    float largura = 20.0f;
    float comprimento = 30.0f;
    float altura = 5.0f;
    std::vector<glm::vec3> trajetoria;

    Drone() = default;
};

#endif