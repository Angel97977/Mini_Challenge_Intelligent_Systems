"""
sim.py

Simulador 2D simple para el Mini Challenge.
Visualiza el XArm y el movimiento de un PuzzleBot.

Autor: equipo
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Importa tu módulo
from xarm_handler import XArm


# =========================
# 🌍 OBJETO SIMULADO
# =========================
class PuzzleBot:
    def __init__(self, pos):
        self.pos = np.array(pos)


# =========================
# 🌍 SIMULADOR
# =========================
class Simulator:
    def __init__(self):
        self.xarm = XArm()
        self.xarm.forward_kinematics()

        # PuzzleBot inicial (sobre ANYmal simulado)
        self.pb = PuzzleBot([0.3, 0.0, 0.0])

        # Zona de destino (mesa)
        self.target = np.array([0.6, 0.3, 0.1])

        # Flags
        self.done = False

        # Para animación
        self.traj = []
        self.generate_trajectory()

    def generate_trajectory(self):
        """
        Genera trayectoria completa (pick + place)
        """
        obj = self.pb.pos
        target = self.target

        # Generamos trayectorias manualmente
        # (para poder animar paso a paso)
        hover = obj + np.array([0, 0, 0.1])
        hover_t = target + np.array([0, 0, 0.1])

        self.traj += self.xarm.plan_cartesian(self.xarm.ee_pos, hover)
        self.traj += self.xarm.plan_cartesian(hover, obj)

        # pick
        self.pick_index = len(self.traj)

        self.traj += self.xarm.plan_cartesian(obj, hover)
        self.traj += self.xarm.plan_cartesian(hover, hover_t)
        self.traj += self.xarm.plan_cartesian(hover_t, target)

        # place
        self.place_index = len(self.traj)

        self.traj += self.xarm.plan_cartesian(target, hover_t)

        self.step = 0

    def update(self, frame):
        if self.step >= len(self.traj):
            return

        p = self.traj[self.step]

        # IK + FK
        q = self.xarm.inverse_kinematics(p)
        self.xarm.q = q
        self.xarm.forward_kinematics()

        # Evento PICK
        if self.step == self.pick_index:
            self.xarm.gripper_close()

        # Evento PLACE
        if self.step == self.place_index:
            self.xarm.gripper_open()

        # Si está agarrando → mueve el PB con el efector
        if self.xarm.holding:
            self.pb.pos = self.xarm.ee_pos.copy()

        self.step += 1

    def draw(self, ax):
        ax.clear()

        # Límites
        ax.set_xlim(-0.1, 1.0)
        ax.set_ylim(-0.5, 0.5)

        # Dibujar efector final
        ee = self.xarm.ee_pos
        ax.scatter(ee[0], ee[1], label="XArm EE")

        # Dibujar PuzzleBot
        ax.scatter(self.pb.pos[0], self.pb.pos[1], label="PuzzleBot")

        # Dibujar objetivo
        ax.scatter(self.target[0], self.target[1], marker="x", label="Target")

        ax.legend()
        ax.set_title("Simulación XArm - Pick & Place")


# =========================
# 🎬 MAIN
# =========================
def main():
    sim = Simulator()

    fig, ax = plt.subplots()

    def animate(frame):
        sim.update(frame)
        sim.draw(ax)

    ani = FuncAnimation(fig, animate, frames=300, interval=50)

    plt.show()


if __name__ == "__main__":
    main()