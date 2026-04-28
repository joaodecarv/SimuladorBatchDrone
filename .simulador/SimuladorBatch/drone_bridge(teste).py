import sys
import math

# Aqui você importaria sua FSM original do ROS 2
# Exemplo: from minha_fsm_ros2 import MinhaFSM
# Para este exemplo, simularemos o retorno de um objeto tipo ROS 2 / PX4

class DroneBridge:
    def __init__(self):
        # Instancie sua FSM aqui
        # self.fsm = MinhaFSM()
        print("Ponte Python-C++ inicializada com sucesso.")

    def update(self, px, py, pz, yaw):
        """
        Recebe dados do C++ (Simulador) e retorna comandos para o C++.
        px, py, pz: Posição atual do drone
        yaw: Orientação atual em radianos
        """
        
        # 1. Chame sua FSM passando o estado atual
        # Ex: msg_ros2 = self.fsm.compute_step(px, py, pz, yaw)
        
        # 2. Simulação de um comando vindo da FSM (TrajectorySetpoint do ROS 2)
        # Supondo que a FSM quer que o drone vá para frente e suba:
        setpoint_vx = 0.5  # m/s
        setpoint_vy = 0.0
        setpoint_vz = 0.2  # subindo
        setpoint_yaw_rate = 0.1

        # 3. TRADUÇÃO: Converte o padrão ROS 2/PX4 para o padrão do seu SimuladorCore
        # Como seu simulador usa [-1.0 a 1.0], normalizamos ou limitamos os valores
        cmd_frente = max(-1.0, min(1.0, setpoint_vx))
        cmd_lado = max(-1.0, min(1.0, setpoint_vy))
        cmd_vertical = max(-1.0, min(1.0, setpoint_vz))
        cmd_giro = max(-1.0, min(1.0, setpoint_yaw_rate))

        # Retorna uma lista de floats que o C++ (pybind11) entende facilmente
        return [cmd_frente, cmd_lado, cmd_vertical, cmd_giro]

# Instância global para ser acessada pelo C++
bridge = DroneBridge()

def get_commands(px, py, pz, yaw):
    return bridge.update(px, py, pz, yaw)