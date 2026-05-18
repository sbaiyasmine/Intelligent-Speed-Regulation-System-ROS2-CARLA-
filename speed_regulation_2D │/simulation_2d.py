import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Paramètres du véhicule ---
dt = 0.05          # pas de temps (secondes)
masse = 1.0        # kg
a_max = 0.047      # accélération max mesurée sur robot réel

# --- PID ---
Kp = 2.0
Ki = 0.5
Kd = 0.1

vitesse_cible = 0.1  # m/s (consigne)

# --- Variables ---
vitesse = 0.0
position = 0.0
erreur_precedente = 0.0
integrale = 0.0

historique_temps = []
historique_vitesse = []
historique_acceleration = []
historique_position = []

# --- Simulation ---
for i in range(200):
    t = i * dt

    # PID
    erreur = vitesse_cible - vitesse
    integrale += erreur * dt
    derivee = (erreur - erreur_precedente) / dt
    commande = Kp * erreur + Ki * integrale + Kd * derivee

    # Limiter à a_max
    acceleration = np.clip(commande, -a_max, a_max)

    # Mise à jour vitesse et position
    vitesse += acceleration * dt
    vitesse = max(0, vitesse)
    position += vitesse * dt
    erreur_precedente = erreur

    historique_temps.append(t)
    historique_vitesse.append(vitesse)
    historique_acceleration.append(acceleration)
    historique_position.append(position)

# --- Affichage ---
fig, axes = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle('Simulation 2D - Régulation de vitesse PID', fontsize=14)

axes[0].plot(historique_temps, historique_vitesse, 'b-', label='Vitesse réelle')
axes[0].axhline(y=vitesse_cible, color='r', linestyle='--', label='Consigne')
axes[0].set_ylabel('Vitesse (m/s)')
axes[0].legend()
axes[0].grid(True)

axes[1].plot(historique_temps, historique_acceleration, 'g-', label='Accélération')
axes[1].axhline(y=a_max, color='r', linestyle='--', label='a_max')
axes[1].axhline(y=-a_max, color='r', linestyle='--')
axes[1].set_ylabel('Accélération (m/s²)')
axes[1].legend()
axes[1].grid(True)

axes[2].plot(historique_temps, historique_position, 'm-', label='Position')
axes[2].set_ylabel('Position (m)')
axes[2].set_xlabel('Temps (s)')
axes[2].legend()
axes[2].grid(True)

plt.tight_layout()
plt.savefig('simulation_2d_result.png')
plt.show()
print("Simulation terminée !")
print(f"Vitesse finale : {historique_vitesse[-1]:.4f} m/s")
print(f"a_max utilisé  : {a_max} m/s²")
