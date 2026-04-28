#include <raylib.h>
#include "../Simulador_Core/SimuladorCore.hpp"
#include "Renderer.hpp"

int main() {
    InitWindow(800, 600, "Simulador Drone ITA - Ubuntu 24.04");
    
    Drone drone;
    SimuladorCore core;
    Commands cmd;
    std::vector<Obstaculo> arena;
    arena.push_back(Obstaculo("Predio", 200, 200, 0, 50, 50, 100));

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if (IsKeyDown(KEY_UP))    cmd.cmdFrente = 1.0f;
        if (IsKeyDown(KEY_DOWN))  cmd.cmdFrente = -1.0f;
        if (IsKeyDown(KEY_LEFT_SHIFT)) cmd.cmdVertical = 1.0f;
        if (IsKeyDown(KEY_LEFT_CONTROL)) cmd.cmdVertical = -1.0f;

        core.atualizar(drone, dt, cmd);

        BeginDrawing();
            ClearBackground(RAYWHITE);
            Renderer::DesenharArena(arena);
            Renderer::DesenharDrone(drone);
            Renderer::DesenharHUD(drone);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}