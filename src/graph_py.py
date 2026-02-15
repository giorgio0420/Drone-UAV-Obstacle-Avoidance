import pandas as pd

import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import numpy as np
import glob
import os

# 1. CERCA IL FILE PIÙ RECENTE
list_of_files = glob.glob('log_*.csv') 
if not list_of_files:
    print("ERRORE: Nessun file log_*.csv trovato nella cartella corrente!")
    print("Assicurati di aver messo questo script nella cartella Downloads.")
    exit()

latest_file = max(list_of_files, key=os.path.getctime)
print(f"Analisi del file: {latest_file}")

# Carica i dati
df = pd.read_csv(latest_file)

# Calcola grandezze derivate
# Errore Distanza (2D)
# + (df['Dz'] - df['Tz'])**2
df['Err_Dist'] = np.sqrt((df['Dx'] - df['Tx'])**2 + (df['Dy'] - df['Ty'])**2)
# Magnitudo Forze
df['Rep_Mag'] = np.sqrt(df['RepFx']**2 + df['RepFy']**2)
df['Vor_Mag'] = np.sqrt(df['VorFx']**2 + df['VorFy']**2)

# ==========================================
# GRAFICO 1: SFORZI DI CONTROLLO
# ==========================================
fig1, ax1 = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
ax1[0].plot(df['Time'], df['Thrust'], color='black')
ax1[0].set_ylabel('Thrust [N]')
ax1[0].set_title('Evoluzione Control Inputs')
ax1[0].grid(True)

ax1[1].plot(df['Time'], df['Roll'], color='red')
ax1[1].set_ylabel('Roll Cmd')
ax1[1].grid(True)

ax1[2].plot(df['Time'], df['Pitch'], color='green')
ax1[2].set_ylabel('Pitch Cmd')
ax1[2].grid(True)

ax1[3].plot(df['Time'], df['Yaw'], color='blue')
ax1[3].set_ylabel('Yaw Cmd')
ax1[3].set_xlabel('Tempo [s]')
ax1[3].grid(True)
plt.tight_layout()


# ==========================================
# GRAFICO 2: ERRORE TRACKING
# ==========================================
plt.figure(figsize=(10, 5))
plt.plot(df['Time'], df['Err_Dist']+1.2, label='Errore Distanza (2D)', color='purple', linewidth=2)
plt.axhline(y=1.2, color='gray', linestyle='--', label='Distanza Desiderata (1.2m)')
plt.ylabel('Distanza [m]')
plt.xlabel('Tempo [s]')
plt.title('Errore di Inseguimento Target')
plt.legend()
plt.grid(True)


# ==========================================
# GRAFICO 3: DISTANZA OSTACOLI E FORZE
# ==========================================
fig3, ax3 = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Distanza Minima
ax3[0].plot(df['Time'], df['ObsDist'], color='orange', linewidth=2)
ax3[0].axhline(y=2.0, color='red', linestyle='--', label='Soglia Attivazione (2.0m)')
ax3[0].set_ylabel('Distanza Minima [m]')
ax3[0].set_title('Rilevamento Ostacoli')
ax3[0].legend()
ax3[0].grid(True)

# Forze (Repulsiva vs Vortice)
ax3[1].plot(df['Time'], df['Rep_Mag'], label='Forza Repulsiva', color='red', alpha=0.8)
ax3[1].plot(df['Time'], df['Vor_Mag'], label='Forza Vortice', color='blue', alpha=0.8)
ax3[1].set_ylabel('Intensità Forza [N]')
ax3[1].set_xlabel('Tempo [s]')
ax3[1].set_title('Evoluzione Forze APF')
ax3[1].legend()
ax3[1].grid(True)
plt.tight_layout()


# ==========================================
# GRAFICO 4: SCENA 2D (Drone vs Ostacoli)
# ==========================================
plt.figure(figsize=(8, 8))
# Carica Ostacoli se esistono
try:
    map_df = pd.read_csv('mappa_ostacoli.csv')
    for _, row in map_df.iterrows():
        circle = plt.Circle((row['X'], row['Y']), row['Radius'], color='gray', alpha=0.5)
        plt.gca().add_patch(circle)
except FileNotFoundError:
    print("Avviso: mappa_ostacoli.csv non trovato. Disegno solo le traiettorie.")

plt.plot(df['Tx'], df['Ty'], label='Target (Bill)', color='green', linestyle='--')
plt.plot(df['Dx'], df['Dy'], label='Drone', color='blue', linewidth=2)
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Traiettoria 2D')
plt.legend()
plt.grid(True)
plt.axis('equal')

plt.show()
