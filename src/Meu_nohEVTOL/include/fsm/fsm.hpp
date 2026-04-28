// =============================================================================
// fsm/fsm.hpp — Implementação do framework FSM
// =============================================================================
// DROP-IN para fsm/include/fsm/fsm.hpp do repositório sae_2025.
// Fornece: fsm::Blackboard, fsm::State, fsm::FSM
//
// API compatível com todos os estados do sae_2025 (fase1, fase2, fase3).
// =============================================================================
#pragma once

#include <any>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fsm {

// ===========================================================================
// Blackboard — quadro negro compartilhado entre os estados da FSM
// ===========================================================================
class Blackboard {
public:
    /**
     * Lê um valor armazenado no blackboard.
     * Retorna T* (ponteiro) — nullptr se a chave não existir ou o tipo for errado.
     * Uso nos estados: auto val = *blackboard.get<float>("takeoff_height");
     */
    template<typename T>
    T* get(const std::string& key) {
        auto it = data_.find(key);
        if (it == data_.end()) return nullptr;
        // Caso 1: valor armazenado diretamente como T
        if (auto* p = std::any_cast<T>(&it->second)) return p;
        // Caso 2: pointer armazenado como T* (via blackboard_set<T>(key, T*))
        if (auto* pp = std::any_cast<T*>(&it->second)) return *pp;
        return nullptr;
    }

    template<typename T>
    const T* get(const std::string& key) const {
        auto it = data_.find(key);
        if (it == data_.end()) return nullptr;
        if (const auto* p = std::any_cast<T>(&it->second)) return p;
        if (const auto* pp = std::any_cast<T*>(&it->second)) return *pp;
        return nullptr;
    }

    /**
     * Escreve um valor no blackboard.
     * Uso nos estados: blackboard.set<int>("current_pole", next_pole);
     */
    template<typename T>
    void set(const std::string& key, T value) {
        data_[key] = std::any(std::move(value));
    }

    /** Verifica se uma chave existe */
    bool has(const std::string& key) const {
        return data_.count(key) > 0;
    }

private:
    std::unordered_map<std::string, std::any> data_;
};

// ===========================================================================
// State — classe base para todos os estados da FSM
// ===========================================================================
class State {
public:
    State() = default;
    virtual ~State() = default;

    /**
     * Chamado UMA vez ao entrar no estado.
     * Inicialize variáveis locais e leia parâmetros do blackboard aqui.
     */
    virtual void on_enter(Blackboard& blackboard) { (void)blackboard; }

    /**
     * Chamado a cada ciclo de execução (chamada de FSM::execute()).
     * Retorne:
     *   ""          → permanecer no estado atual (sem transição)
     *   "CONDICAO"  → disparar transição definida em add_transitions()
     */
    virtual std::string act(Blackboard& blackboard) { (void)blackboard; return ""; }

    /**
     * Chamado UMA vez ao sair do estado.
     * Limpe recursos e escreva resultados no blackboard se necessário.
     */
    virtual void on_exit(Blackboard& blackboard) { (void)blackboard; }
};

// ===========================================================================
// FSM — gerenciador de estados finitos
// ===========================================================================
class FSM {
public:
    /**
     * Construtor.
     * terminal_states: lista de nomes de estados terminais (ex.: {"ERROR","FINISHED"})
     * A FSM começa no PRIMEIRO estado adicionado com add_state().
     */
    explicit FSM(std::initializer_list<std::string> terminal_states) {
        for (const auto& s : terminal_states) {
            terminal_states_.insert(s);
        }
    }

    explicit FSM(std::vector<std::string> terminal_states) {
        for (const auto& s : terminal_states) {
            terminal_states_.insert(s);
        }
    }

    virtual ~FSM() = default;

    // -----------------------------------------------------------------------
    // Configuração (chamada no construtor da subclasse)
    // -----------------------------------------------------------------------

    /** Adiciona um estado à FSM. O primeiro adicionado é o estado inicial. */
    void add_state(const std::string& name, std::unique_ptr<State> state) {
        if (states_.empty()) {
            current_state_ = name;  // primeiro estado = estado inicial
        }
        states_[name] = std::move(state);
        // Garante que a entrada de transições existe
        if (transitions_.find(name) == transitions_.end()) {
            transitions_[name] = {};
        }
    }

    /**
     * Define transições a partir de um estado.
     * Uso: add_transitions("ESTADO", {{"CONDICAO", "PROXIMO_ESTADO"}, ...});
     */
    void add_transitions(const std::string& from_state,
                         std::vector<std::pair<std::string, std::string>> transitions) {
        transitions_[from_state] = std::move(transitions);
    }

    /** Escreve um valor no blackboard (atalho para uso no construtor da FSM) */
    template<typename T>
    void blackboard_set(const std::string& key, T value) {
        blackboard_.set<T>(key, std::move(value));
    }

    /**
     * Sobrecarga para ponteiros: blackboard_set<Drone>("drone", drone_ptr)
     * Armazena o ponteiro T* permitindo get<T>("drone") retornar T*.
     */
    template<typename T>
    void blackboard_set(const std::string& key, T* value) {
        blackboard_.set<T*>(key, value);
    }

    // -----------------------------------------------------------------------
    // Execução
    // -----------------------------------------------------------------------

    /**
     * Executa um ciclo da FSM:
     *  1. Se for o primeiro ciclo, chama on_enter() do estado atual
     *  2. Chama act() do estado atual
     *  3. Se o resultado corresponder a uma transição: efetua a troca de estado
     */
    void execute() {
        if (finished_) return;

        // Se é o primeiro tick do estado corrente, chama on_enter
        if (!entered_) {
            auto it = states_.find(current_state_);
            if (it != states_.end()) {
                it->second->on_enter(blackboard_);
            }
            entered_ = true;
        }

        auto it = states_.find(current_state_);
        if (it == states_.end()) {
            std::cerr << "[FSM] Estado desconhecido: " << current_state_ << "\n";
            finished_ = true;
            return;
        }

        // Executa act()
        std::string result = it->second->act(blackboard_);

        // Sem transição: permanece no estado
        if (result.empty()) return;

        // Busca transição
        auto trans_it = transitions_.find(current_state_);
        if (trans_it != transitions_.end()) {
            for (const auto& [condition, next_state] : trans_it->second) {
                if (condition == result) {
                    // Chama on_exit do estado corrente
                    it->second->on_exit(blackboard_);

                    std::string prev = current_state_;
                    current_state_  = next_state;
                    entered_        = false;

                    // Verifica se chegou a um estado terminal
                    if (terminal_states_.count(current_state_) > 0) {
                        finished_ = true;
                        std::cout << "[FSM] Terminal: " << current_state_
                                  << " (veio de " << prev << " via '" << result << "')\n";
                        return;
                    }

                    // Chama on_enter do novo estado
                    auto next_it = states_.find(current_state_);
                    if (next_it != states_.end()) {
                        next_it->second->on_enter(blackboard_);
                        entered_ = true;
                    }
                    return;
                }
            }
        }

        // Condição não mapeada — log de aviso
        std::cerr << "[FSM] Condicao '" << result << "' nao mapeada em '"
                  << current_state_ << "'. Ignorando.\n";
    }

    /** Retorna true se a FSM está em um estado terminal */
    bool is_finished() const { return finished_; }

    /** Retorna o nome do estado atual */
    const std::string& current_state() const { return current_state_; }

    /** Acesso ao blackboard (para o batch_simulator ler/escrever parâmetros) */
    Blackboard& blackboard() { return blackboard_; }
    const Blackboard& blackboard() const { return blackboard_; }

protected:
    Blackboard blackboard_;

private:
    std::map<std::string, std::unique_ptr<State>>               states_;
    std::map<std::string, std::vector<std::pair<std::string, std::string>>> transitions_;
    std::set<std::string>                                        terminal_states_;
    std::string current_state_{};
    bool        entered_{false};
    bool        finished_{false};
};

}  // namespace fsm
