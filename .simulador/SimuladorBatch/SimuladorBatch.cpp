#include <pybind11/embed.h>
#include <pybind11/stl.h> 


#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg>




#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <fstream>    // NECESSÁRIO para salvar os CSVs
#include <filesystem> // NECESSÁRIO para criar a pasta logs
#include "../Simulador_Core/SimuladorCore.hpp"
#include "../objetos/Drone.hpp"
#include "../objetos/Obstaculo.hpp"

// Define o apelido para não dar erro de "py has not been declared"
namespace py = pybind11;

// Define os mutexes que o compilador não encontrou
std::mutex mtx;
std::mutex mtx_log;

void salvarDadosTrajetoria(int id, const Drone& drone, bool colidiu) {
    // Cria a pasta de logs se não existir
    std::filesystem::create_directories("logs");
    
    std::ofstream arquivo("logs/sim_res_" + std::to_string(id) + ".csv");
    if (arquivo.is_open()) {
        arquivo << "x,y,z,status\n";
        std::string status = colidiu ? "falha" : "sucesso";
        for (const auto& p : drone.trajetoria) {
            arquivo << p.x << "," << p.y << "," << p.z << "," << status << "\n";
        }
        arquivo.close();
    }
}

// ... (mantenha os mesmos #includes e namespaces)

void executarSimulacao(int id, std::vector<Obstaculo> arena, const py::object& get_commands) {
    try {
        // Adquire o GIL para a thread
        py::gil_scoped_acquire acquire; 
        
        Drone meuDrone;
        SimuladorCore core;
        Commands cmd;
        float tempo = 0.0f;
        const float dt = 0.016f;
        bool colidiu = false;

        while (tempo < 30.0f && !colidiu) {
            // Chamada da FSM
            py::object result = get_commands(
                meuDrone.posicao.x, meuDrone.posicao.y, meuDrone.posicao.z, meuDrone.yaw
            );

            auto lista_cmds = result.cast<std::vector<float>>();
            cmd.cmdFrente   = lista_cmds[0];
            cmd.cmdLado     = lista_cmds[1];
            cmd.cmdVertical = lista_cmds[2];
            cmd.cmdGiro     = lista_cmds[3];

            // Libera o GIL para a física pesada em C++
            {
                py::gil_scoped_release release;
                core.atualizar(meuDrone, dt, cmd);
                if (core.checarColisao(meuDrone, arena)) colidiu = true;
                tempo += dt;
            }
        }
        
        salvarDadosTrajetoria(id, meuDrone, colidiu);
        
        std::lock_guard<std::mutex> lock(mtx_log);
        std::cout << "Simulacao " << id << (colidiu ? " [COLIDIU]" : " [SUCESSO]") << std::endl;

        // O 'acquire' será destruído aqui, liberando o GIL. 
        // Como 'get_commands' é uma referência constante, a thread NÃO tentará destruí-lo.
    } catch (py::error_already_set &e) {
        std::lock_guard<std::mutex> lock(mtx_log);
        std::cerr << "Erro no Python: " << e.what() << std::endl;
    }
}

int main() {
    py::scoped_interpreter guard{}; 

    py::object get_commands;
    try {
        py::module_ fsm_bridge = py::module_::import("drone_bridge");
        get_commands = fsm_bridge.attr("get_commands");
    } catch (py::error_already_set &e) {
        std::cerr << "Erro: " << e.what() << std::endl;
        return 1;
    }

    int total = 5; 
    std::vector<Obstaculo> arena = { Obstaculo("Predio", 200, 200, 0, 50, 50, 100) };
    std::vector<std::thread> threads;

    // CRIAMOS AS THREADS AINDA COM O GIL SEGURO
    for (int i = 0; i < total; ++i) {
        threads.emplace_back(executarSimulacao, i, arena, std::ref(get_commands));
    }

    // AGORA LIBERAMOS O GIL para que as threads possam adquiri-lo
    py::gil_scoped_release release; 

    for (auto& t : threads) t.join();
    return 0;
}