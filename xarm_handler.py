"""
xarm_handler.py

Módulo simplificado de manipulación para un xArm (3 DoF equivalente)
usado en el Mini Challenge.

Funcionalidades:
- Forward / Inverse Kinematics
- Planeación cartesiana (interpolación lineal)
- Ejecución de trayectorias
- Gripper simple (open/close)
- Secuencias de pick & place

Autor: equipo
"""

import numpy as np


class XArm:
    def __init__(self, l2=0.20, l3=0.20, z_offset=0.0):
        """
        Brazo 3 DoF simplificado:
        q1: rotación base (yaw)
        q2: hombro
        q3: codo
        """
        self.l2 = l2
        self.l3 = l3
        self.z_offset = z_offset  # altura base si la necesitas

        self.q = np.zeros(3)
        self.ee_pos = np.zeros(3)

        self.holding = False  # estado del gripper

    # =========================
    # 🔧 CINEMÁTICA
    # =========================
    def forward_kinematics(self, q=None):
        """
        Calcula la posición del efector final (x, y, z)
        """
        if q is not None:
            self.q = q

        q1, q2, q3 = self.q

        r = self.l2 * np.cos(q2) + self.l3 * np.cos(q2 + q3)
        z = self.l2 * np.sin(q2) + self.l3 * np.sin(q2 + q3) + self.z_offset

        x = r * np.cos(q1)
        y = r * np.sin(q1)

        self.ee_pos = np.array([x, y, z])
        return self.ee_pos

    def inverse_kinematics(self, p):
        """
        IK geométrica cerrada para brazo 3DoF
        """
        x, y, z = p
        z = z - self.z_offset

        # Base
        q1 = np.arctan2(y, x)

        r = np.sqrt(x**2 + y**2)

        # Ley de cosenos
        D = (r**2 + z**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)

        # Clamp numérico
        D = np.clip(D, -1.0, 1.0)

        q3 = np.arctan2(-np.sqrt(1 - D**2), D)  # elbow-down

        q2 = np.arctan2(z, r) - np.arctan2(
            self.l3 * np.sin(q3),
            self.l2 + self.l3 * np.cos(q3)
        )

        return np.array([q1, q2, q3])

    # =========================
    # 📈 PLANEACIÓN
    # =========================
    def plan_cartesian(self, p_start, p_end, steps=30):
        """
        Interpolación lineal en espacio cartesiano
        """
        traj = []
        for t in np.linspace(0, 1, steps):
            p = (1 - t) * p_start + t * p_end
            traj.append(p)
        return traj

    # =========================
    # ⚙️ EJECUCIÓN
    # =========================
    def execute(self, traj):
        """
        Ejecuta trayectoria cartesiana usando IK
        """
        for p in traj:
            q = self.inverse_kinematics(p)
            self.q = q
            self.forward_kinematics()

    # =========================
    # ✋ GRIPPER
    # =========================
    def gripper_close(self):
        self.holding = True

    def gripper_open(self):
        self.holding = False

    # =========================
    # 📦 PICK & PLACE
    # =========================
    def pick(self, obj_pos, hover_height=0.10):
        """
        Secuencia de agarre:
        1. Ir arriba del objeto
        2. Bajar
        3. Cerrar gripper
        4. Subir
        """
        obj_pos = np.array(obj_pos)
        above = obj_pos + np.array([0, 0, hover_height])

        # Ir arriba
        self.execute(self.plan_cartesian(self.ee_pos, above))

        # Bajar
        self.execute(self.plan_cartesian(above, obj_pos))

        # Agarrar
        self.gripper_close()

        # Subir
        self.execute(self.plan_cartesian(obj_pos, above))

    def place(self, target_pos, hover_height=0.10):
        """
        Secuencia de colocación:
        1. Ir arriba
        2. Bajar
        3. Abrir gripper
        4. Subir
        """
        target_pos = np.array(target_pos)
        above = target_pos + np.array([0, 0, hover_height])

        # Ir arriba
        self.execute(self.plan_cartesian(self.ee_pos, above))

        # Bajar
        self.execute(self.plan_cartesian(above, target_pos))

        # Soltar
        self.gripper_open()

        # Subir
        self.execute(self.plan_cartesian(target_pos, above))

    def execute_pick_and_place(self, obj_pos, target_pos):
        """
        Pipeline completo
        """
        self.pick(obj_pos)
        self.place(target_pos)

    # =========================
    # 🧪 DEBUG / TEST
    # =========================
    def get_state(self):
        return {
            "q": self.q,
            "ee_pos": self.ee_pos,
            "holding": self.holding
        }


# =========================
# 🧪 TEST LOCAL
# =========================
if __name__ == "__main__":
    arm = XArm()

    # Posición inicial del efector
    arm.forward_kinematics()

    obj = np.array([0.3, 0.0, 0.0])
    target = np.array([0.5, 0.2, 0.1])

    print("Inicio:", arm.get_state())

    arm.execute_pick_and_place(obj, target)

    print("Final:", arm.get_state())