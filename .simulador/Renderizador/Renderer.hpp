#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "raylib.h"
#include "Drone.hpp"
#include "Obstaculo.hpp"
#include <vector>

class Renderer {
public:
    static void DesenharDrone(const Drone& drone);
    static void DesenharArena(const std::vector<Obstaculo>& obs);
    static void DesenharHUD(const Drone& drone);
};

#endif