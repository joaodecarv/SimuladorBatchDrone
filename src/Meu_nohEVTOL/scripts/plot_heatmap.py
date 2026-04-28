#!/usr/bin/env python3
"""
plot_heatmap.py
---------------
Gera gráficos 2D das trajetórias dos drones a partir do CSV
gerado pelo batch_simulator.

Formato esperado do CSV (colunas):
  drone_id, step, sim_time, x, y, altitude, state,
  landed_on_base, base_id, collided, collision_info

Uso:
    python3 plot_heatmap.py sim_output.csv
    python3 plot_heatmap.py sim_output.csv --output heatmap.png
    python3 plot_heatmap.py sim_output.csv --mode altitude

Instalar dependências:
    pip install matplotlib numpy pandas
"""

import argparse
import sys
import os

try:
    import numpy as np
    import pandas as pd
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
except ImportError as e:
    print(f"[plot_heatmap] Dependência faltando: {e}")
    print("  pip install matplotlib numpy pandas")
    sys.exit(1)


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Garante que colunas opcionais existam mesmo em CSVs antigos
    for col in ("collided", "collision_info", "landed_on_base", "base_id"):
        if col not in df.columns:
            df[col] = None
    return df


def plot_heatmap(df: pd.DataFrame, output: str | None = None) -> None:
    """Mapa de calor: densidade de pontos no plano XY (visão de cima)."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Heatmap de Trajetórias – Visão Superior", fontsize=14)

    # --- Painel esquerdo: heatmap 2D de densidade ---
    ax = axes[0]
    h = ax.hist2d(df["x"], df["y"],
                  bins=60, cmap="hot_r", norm=mcolors.LogNorm())
    fig.colorbar(h[3], ax=ax, label="Contagem (log)")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Densidade de Pontos")
    ax.set_aspect("equal")

    # --- Painel direito: trajetórias por drone ---
    ax2 = axes[1]
    cmap = plt.get_cmap("tab20")
    for i, (drone_id, grp) in enumerate(df.groupby("drone_id")):
        color = cmap(i % 20)
        ax2.plot(grp["x"], grp["y"],
                 "-", linewidth=0.8, color=color, label=f"Drone {drone_id}")
        # Marca início (triângulo) e fim (quadrado)
        ax2.plot(grp["x"].iloc[0], grp["y"].iloc[0],
                 "^", color=color, markersize=8, zorder=5)
        ax2.plot(grp["x"].iloc[-1], grp["y"].iloc[-1],
                 "s", color=color, markersize=8, zorder=5)
        # Marca colisão com X vermelho
        col_rows = grp[grp["collided"] == True]
        if not col_rows.empty:
            ax2.plot(col_rows["x"].iloc[-1], col_rows["y"].iloc[-1],
                     "x", color="red", markersize=14, markeredgewidth=2.5,
                     zorder=6, label=f"Colisão d{drone_id}")

    ax2.set_xlabel("X [m]")
    ax2.set_ylabel("Y [m]")
    ax2.set_title("Trajetórias  (▲ início  ■ fim  ✕ colisão)")
    ax2.legend(fontsize=6, ncol=2)
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    if output:
        plt.savefig(output, dpi=150, bbox_inches="tight")
        print(f"[plot_heatmap] Imagem salva em: {output}")
    else:
        plt.show()


def plot_altitude_profile(df: pd.DataFrame, output: str | None = None) -> None:
    """Perfil de altitude ao longo do tempo de cada drone."""
    fig, ax = plt.subplots(figsize=(12, 5))
    cmap = plt.get_cmap("tab20")
    for i, (drone_id, grp) in enumerate(df.groupby("drone_id")):
        color = cmap(i % 20)
        ax.plot(grp["sim_time"], grp["altitude"],
                linewidth=1.2, color=color, label=f"Drone {drone_id}")
        # Linha vertical no instante da colisão
        col_rows = grp[grp["collided"] == True]
        if not col_rows.empty:
            ax.axvline(col_rows["sim_time"].iloc[-1], color=color,
                       linestyle="--", linewidth=1, alpha=0.8)
    ax.set_xlabel("Tempo simulado (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Perfil de Altitude por Drone  (-- = colisão)")
    ax.legend(fontsize=7, ncol=4)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    out = (output.replace(".png", "_altitude.png") if output else None)
    if out:
        plt.savefig(out, dpi=150, bbox_inches="tight")
        print(f"[plot_heatmap] Perfil de altitude salvo em: {out}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Heatmap de trajetórias EVTOL")
    parser.add_argument("csv", help="Arquivo CSV gerado pelo batch_simulator")
    parser.add_argument("--output", "-o", default=None,
                        help="Salva em arquivo PNG (omite exibição)")
    parser.add_argument("--mode", choices=["all", "heatmap", "trajectories", "altitude"],
                        default="all", help="Tipo de gráfico")
    args = parser.parse_args()

    if not os.path.isfile(args.csv):
        print(f"[plot_heatmap] Arquivo não encontrado: {args.csv}")
        sys.exit(1)

    df = load_csv(args.csv)
    print(f"[plot_heatmap] Carregados {len(df)} pontos de "
          f"{df['drone_id'].nunique()} drones.")

    # Resumo de colisões
    collided = df[df["collided"] == True].groupby("drone_id")["collision_info"].last()
    if not collided.empty:
        print("[plot_heatmap] Colisões detectadas:")
        for did, info in collided.items():
            print(f"  Drone {did}: {info}")

    if args.mode in ("all", "heatmap", "trajectories"):
        plot_heatmap(df, args.output)

    if args.mode in ("all", "altitude"):
        plot_altitude_profile(df, args.output)


if __name__ == "__main__":
    main()
