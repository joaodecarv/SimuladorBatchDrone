#ifndef OBSTACULO_HPP
#define OBSTACULO_HPP

#include <glm/glm.hpp>
#include <string>

struct Obstaculo {
    glm::vec3 posicao;
    float largura, comprimento, altura;
    std::string nome;

    Obstaculo(std::string n, float x, float y, float z, float l, float c, float h)
        : nome(n), posicao(x, y, z), largura(l), comprimento(c), altura(h) {}
};

#endif