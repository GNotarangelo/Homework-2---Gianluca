import os
import numpy as np
import matplotlib.pyplot as plt

# --- Configurazione Controller e Percorsi ---
# La cartella di log *deve* essere definita qui, ad esempio:
# log_folder = "/path/alla/tua/cartella/log" 
# **DEVI SOSTITUIRE QUESTA LINEA CON IL PERCORSO REALE ALLA TUA CARTELLA LOG**
try:
    # Definizione di un percorso di esempio. Sostituiscilo!
    log_folder = os.path.join(os.getcwd(), 'logs') 
    
    if not os.path.isdir(log_folder):
        print(f"--- ERRORE ---")
        print(f"La cartella specificata non esiste: {log_folder}")
        print("Assicurati che il percorso sia corretto e di avere i permessi.")
        exit() 
except Exception as e:
    print(f"Error in defining path : {e}")
    exit()

# Nome del controller da plottare
controller_name = "vision"

# --- Caricamento Dati ---
try:
    # Costruzione dei percorsi dei file
    pos_path = os.path.join(log_folder, f"{controller_name}_positions.csv")
    vel_path = os.path.join(log_folder, f"{controller_name}_velocities.csv")
    
    # Caricamento dati dai file
    print(f"Caricamento dati per il controller: {controller_name}")
    pos_data = np.loadtxt(pos_path, delimiter=',', skiprows=1)
    vel_data = np.loadtxt(vel_path, delimiter=',', skiprows=1)
    
except IOError as e:
    print(f"--- ERRORE ---")
    print(f"File non trovato. Sei sicuro che i file .csv siano in:")
    print(f"{log_folder} ?")
    print(f"Dettaglio Errore: {e}")
    exit()

# --- Preparazione Dati per il Plot ---
time = pos_data[:, 0]
q = pos_data[:, 1:]
q_dot = vel_data[:, 1:]

num_joints = q.shape[1] # Numero di giunti (dovrebbe essere 7)

# --- Plot delle Posizioni ---
fig_pos, axs_pos = plt.subplots(num_joints, 1, figsize=(10, 15), sharex=True)
fig_pos.suptitle(f'Posizioni Giunti - Controller: {controller_name}', fontsize=16)

for i in range(num_joints):
    axs_pos[i].plot(time, q[:, i], label=f'Posizione Giunto {i+1}')
    axs_pos[i].set_ylabel(f'q{i+1} (rad)')
    axs_pos[i].grid(True)
    axs_pos[i].legend()

axs_pos[-1].set_xlabel('Tempo (s)')

# --- Plot delle Velocità ---
fig_vel, axs_vel = plt.subplots(num_joints, 1, figsize=(10, 15), sharex=True)
fig_vel.suptitle(f'Velocità Comandate Giunti - Controller: {controller_name}', fontsize=16)

for i in range(num_joints):
    axs_vel[i].plot(time, q_dot[:, i], label=f'Velocità Giunto {i+1}')
    axs_vel[i].set_ylabel(f'q_dot{i+1} (rad/s)')
    axs_vel[i].grid(True)
    axs_vel[i].legend()

axs_vel[-1].set_xlabel('Tempo (s)')

plt.tight_layout()
plt.show()
