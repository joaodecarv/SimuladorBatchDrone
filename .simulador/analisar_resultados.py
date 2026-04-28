import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

def gerar_analise(diretorio="logs"):
    plt.figure(figsize=(10, 8))
    arquivos = glob.glob(os.path.join(diretorio, "*.csv"))
    
    if not arquivos:
        print("Logs não encontrados!")
        return

    for arq in arquivos:
        df = pd.read_csv(arq)
        status = df['status'].iloc[0]
        cor = 'green' if status == 'sucesso' else 'red'
        alpha = 0.1 if status == 'sucesso' else 0.4
        plt.plot(df['x'], df['y'], color=cor, alpha=alpha)

    plt.gca().invert_yaxis() # Padrão Raylib/Screen
    plt.title("Heatmap de Trajetórias (Verde: Sucesso | Vermelho: Falha)")
    plt.savefig("heatmap_consolidado.png")
    plt.show()

if __name__ == "__main__":
    gerar_analise()