"""
Simule la stabilisation en tangage d'un axe de rotation face à une consigne
en échelon, avec un correcteur PID. Génère un graphique de la réponse.
"""

import os
import numpy as np
import matplotlib.pyplot as plt

from sim.dynamics import RotationalAxis
from sim.controller import PID


def run_simulation(
    duration=5.0,
    dt=0.01,
    inertia=0.02,      # kg*m^2, ordre de grandeur petit quadrirotor
    damping=0.02,
    setpoint_deg=15.0,
    kp=0.15,
    ki=0.05,
    kd=0.02,
    torque_max=0.5,    # N*m
):
    axis = RotationalAxis(inertia=inertia, damping=damping)
    pid = PID(kp=kp, ki=ki, kd=kd, output_limit=torque_max)

    setpoint = np.radians(setpoint_deg)
    n_steps = int(duration / dt)

    t_history = np.zeros(n_steps)
    theta_history = np.zeros(n_steps)
    torque_history = np.zeros(n_steps)

    for i in range(n_steps):
        t = i * dt
        error = setpoint - axis.theta
        torque = pid.compute(error, dt)
        theta, _ = axis.step(torque, dt)

        t_history[i] = t
        theta_history[i] = np.degrees(theta)
        torque_history[i] = torque

    return t_history, theta_history, torque_history, setpoint_deg


def plot_results(t, theta_deg, torque, setpoint_deg, output_path):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(t, theta_deg, label="Angle de tangage")
    ax1.axhline(setpoint_deg, color="red", linestyle="--", label="Consigne")
    ax1.set_ylabel("Angle (degrés)")
    ax1.set_title("Réponse en tangage — correcteur PID")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, torque, color="darkorange", label="Couple commandé")
    ax2.set_xlabel("Temps (s)")
    ax2.set_ylabel("Couple (N·m)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=150)
    plt.close()


if __name__ == "__main__":
    t, theta_deg, torque, setpoint_deg = run_simulation()
    plot_results(t, theta_deg, torque, setpoint_deg, "results/attitude_response.png")

    # Quelques métriques simples de performance, affichées en console
    overshoot = max(theta_deg) - setpoint_deg
    final_error = abs(setpoint_deg - theta_deg[-1])
    print(f"Dépassement max : {overshoot:.2f} degrés")
    print(f"Erreur statique finale : {final_error:.3f} degrés")
    print("Graphique sauvegardé dans results/attitude_response.png")
