#include "Renderer.hpp"
#include <cmath>

void Renderer::DesenharDrone(const Drone& drone) {
    float offsetSombra = drone.posicao.z * 0.5f;
    // Sombra
    DrawRectanglePro({drone.posicao.x + offsetSombra, drone.posicao.y + offsetSombra, drone.comprimento, drone.largura},
                     {drone.comprimento/2, drone.largura/2}, drone.yaw * RAD2DEG, Fade(GRAY, 0.4f));
    // Corpo
    DrawRectanglePro({drone.posicao.x, drone.posicao.y, drone.comprimento, drone.largura},
                     {drone.comprimento/2, drone.largura/2}, drone.yaw * RAD2DEG, SKYBLUE);
    // Indicador Frontal
    Vector2 frente = {drone.posicao.x + sinf(drone.yaw) * (drone.comprimento/2),
                      drone.posicao.y - cosf(drone.yaw) * (drone.comprimento/2)};
    DrawCircleV(frente, 5, RED);
}

void Renderer::DesenharArena(const std::vector<Obstaculo>& obs) {
    for (const auto& o : obs) {
        DrawRectangleLines(o.posicao.x - o.largura/2, o.posicao.y - o.comprimento/2, o.largura, o.comprimento, DARKGRAY);
    }
}

void Renderer::DesenharHUD(const Drone& drone) {
    DrawRectangle(10, 10, 200, 100, Fade(BLACK, 0.5f));
    DrawText(TextFormat("Z: %.2f", drone.posicao.z), 20, 20, 20, YELLOW);
    DrawText(TextFormat("Yaw: %.2f", drone.yaw), 20, 50, 20, WHITE);
}