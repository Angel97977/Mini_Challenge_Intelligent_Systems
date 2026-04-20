"""
husky_pusher.py
===============
Fase 1 — Almacén Robótico Colaborativo (TE3002B)
Responsable: Nabor Sebastian Toro García

Descripción:
    Simulación del Husky UGV con modelo skid-steer de 4 ruedas.
    El robot empuja 3 cajas grandes fuera del corredor (6 x 2 m)
    usando un LiDAR 2D simulado y un planificador local simple.

Estado actual:
    - [x] Modelo cinemático skid-steer
    - [x] Simulación de LiDAR 2D (raycast simple)
    - [x] Representación de cajas y corredor
    - [ ] Planificador local (en progreso)
    - [ ] Compensación de deslizamiento
    - [ ] Integración con coordinator.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# ---------------------------------------------------------------------------
# Parámetros del robot y del entorno
# ---------------------------------------------------------------------------

WHEEL_BASE   = 0.55   # [m] distancia entre ejes izquierdo y derecho (Husky real)
MAX_VEL      = 0.5    # [m/s] velocidad lineal máxima
MAX_OMEGA    = 1.0    # [rad/s] velocidad angular máxima
DT           = 0.05   # [s]  paso de simulación (20 Hz)

CORRIDOR_W   = 2.0    # [m] ancho del corredor
CORRIDOR_L   = 6.0    # [m] largo del corredor

BOX_SIZE     = 0.4    # [m] lado de cada caja (cuadradas en 2D)
NUM_BOXES    = 3

LIDAR_RANGE  = 4.0    # [m] alcance máximo del LiDAR
LIDAR_RAYS   = 180    # número de rayos (resolución angular 1°)

SLIP_FACTOR  = 0.05   # s — factor de deslizamiento del terreno (pendiente futura)


# ---------------------------------------------------------------------------
# Clase principal del Husky
# ---------------------------------------------------------------------------

class HuskyPusher:
    """
    Modelo cinemático skid-steer del Husky + LiDAR 2D simulado.

    Estado interno:
        x, y   — posición en el plano [m]
        theta  — orientación [rad]
    """

    def __init__(self, x0=0.5, y0=1.0, theta0=0.0):
        # Pose inicial
        self.x     = x0
        self.y     = y0
        self.theta = theta0

        # Historial para graficar trayectoria
        self.traj_x = [x0]
        self.traj_y = [y0]

        # Estado de tarea (para coordinator.py)
        self.task_status = "en_progreso"   # "en_progreso" | "tarea_completada" | "error"
        self.boxes_pushed = 0

    # ------------------------------------------------------------------
    # Cinemática skid-steer
    # ------------------------------------------------------------------

    def skid_steer_model(self, v_left, v_right):
        """
        Actualiza la pose del robot dados los comandos de velocidad
        de las ruedas izquierda y derecha.

        Ecuaciones:
            v     = (v_right + v_left) / 2
            omega = (v_right - v_left) / WHEEL_BASE
            x    += v * cos(theta) * dt
            y    += v * sin(theta) * dt
            theta += omega * dt
        """
        v_left  = np.clip(v_left,  -MAX_VEL, MAX_VEL)
        v_right = np.clip(v_right, -MAX_VEL, MAX_VEL)

        v     = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / WHEEL_BASE

        self.x     += v * np.cos(self.theta) * DT
        self.y     += v * np.sin(self.theta) * DT
        self.theta += omega * DT

        # Normalizar ángulo a [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        self.traj_x.append(self.x)
        self.traj_y.append(self.y)

    def get_pose(self):
        """Retorna la pose actual como dict (interfaz para coordinator.py)."""
        return {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "theta": round(self.theta, 4),
            "task_status": self.task_status,
            "boxes_pushed": self.boxes_pushed,
        }

    # ------------------------------------------------------------------
    # LiDAR 2D simulado (raycast AABB)
    # ------------------------------------------------------------------

    def simulate_lidar(self, obstacles):
        """
        Simula lecturas del LiDAR 2D lanzando LIDAR_RAYS rayos en 360°.

        Args:
            obstacles: lista de dicts con {"x", "y", "w", "h"}
                       representando cajas como rectángulos alineados al eje.

        Returns:
            ranges: np.array de distancias medidas (LIDAR_RANGE si no hay obstáculo)
            angles: np.array de ángulos correspondientes [rad]
        """
        angles = np.linspace(self.theta, self.theta + 2 * np.pi,
                             LIDAR_RAYS, endpoint=False)
        ranges = np.full(LIDAR_RAYS, LIDAR_RANGE)

        for i, angle in enumerate(angles):
            dx = np.cos(angle)
            dy = np.sin(angle)

            for obs in obstacles:
                dist = self._ray_aabb(self.x, self.y, dx, dy,
                                      obs["x"], obs["y"], obs["w"], obs["h"])
                if dist is not None and dist < ranges[i]:
                    ranges[i] = dist

        return ranges, angles

    @staticmethod
    def _ray_aabb(ox, oy, dx, dy, bx, by, bw, bh):
        """
        Intersección rayo–AABB (Axis-Aligned Bounding Box).
        Retorna la distancia al primer impacto, o None si no hay intersección.
        """
        t_min, t_max = -np.inf, np.inf

        for o, d, b_lo, b_hi in [
            (ox, dx, bx, bx + bw),
            (oy, dy, by, by + bh),
        ]:
            if abs(d) < 1e-9:
                if o < b_lo or o > b_hi:
                    return None
            else:
                t1 = (b_lo - o) / d
                t2 = (b_hi - o) / d
                t_min = max(t_min, min(t1, t2))
                t_max = min(t_max, max(t1, t2))

        if t_max < 0 or t_min > t_max:
            return None
        t = t_min if t_min >= 0 else t_max
        return t if t >= 0 else None

    # ------------------------------------------------------------------
    # Planificador local (stub — en desarrollo)
    # ------------------------------------------------------------------

    def compute_control(self, target_x, target_y, obstacles):
        """
        Planificador local simple: go-to-goal con frenado por LiDAR.

        TODO (próxima iteración):
            - Agregar campo potencial repulsivo para obstáculos
            - Compensar deslizamiento con factor SLIP_FACTOR
            - Confirmar llegada y actualizar boxes_pushed

        Returns:
            v_left, v_right — comandos de velocidad para skid_steer_model()
        """
        ranges, _ = self.simulate_lidar(obstacles)
        min_dist   = np.min(ranges)

        # Dirección al objetivo
        dx    = target_x - self.x
        dy    = target_y - self.y
        dist  = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)
        heading_error = (angle_to_goal - self.theta + np.pi) % (2 * np.pi) - np.pi

        # Velocidad lineal (reducir si hay obstáculo cerca)
        speed_scale = min(1.0, min_dist / 0.5)   # frenado si <0.5 m
        v_base      = MAX_VEL * speed_scale * min(1.0, dist)

        # Velocidad angular proporcional al error de rumbo
        k_omega    = 2.0
        omega_cmd  = np.clip(k_omega * heading_error, -MAX_OMEGA, MAX_OMEGA)

        # Convertir a velocidades de rueda
        v_right = v_base + omega_cmd * WHEEL_BASE / 2.0
        v_left  = v_base - omega_cmd * WHEEL_BASE / 2.0

        # ¿Llegó al objetivo?
        if dist < 0.15:
            self.boxes_pushed += 1
            if self.boxes_pushed >= NUM_BOXES:
                self.task_status = "tarea_completada"

        return v_left, v_right


# ---------------------------------------------------------------------------
# Visualización rápida (demo)
# ---------------------------------------------------------------------------

def run_demo():
    """
    Demo: Husky navega hacia las 3 cajas y las saca del corredor.
    Muestra trayectoria + lecturas LiDAR en matplotlib.
    """
    # Cajas dentro del corredor
    boxes = [
        {"x": 1.5, "y": 0.5, "w": BOX_SIZE, "h": BOX_SIZE},
        {"x": 3.0, "y": 0.9, "w": BOX_SIZE, "h": BOX_SIZE},
        {"x": 4.5, "y": 0.3, "w": BOX_SIZE, "h": BOX_SIZE},
    ]
    targets = [(b["x"] + b["w"] / 2, b["y"] + b["h"] / 2) for b in boxes]

    robot = HuskyPusher(x0=0.3, y0=1.0, theta0=0.0)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_xlim(-0.5, CORRIDOR_L + 0.5)
    ax.set_ylim(-0.5, CORRIDOR_W + 0.5)
    ax.set_aspect("equal")
    ax.set_title("Husky — Simulación 2D (Fase 1)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    # Dibujar corredor
    corridor = patches.Rectangle((0, 0), CORRIDOR_L, CORRIDOR_W,
                                  linewidth=2, edgecolor="black",
                                  facecolor="lightyellow", label="Corredor")
    ax.add_patch(corridor)

    # Dibujar cajas
    for b in boxes:
        rect = patches.Rectangle((b["x"], b["y"]), b["w"], b["h"],
                                  linewidth=1, edgecolor="saddlebrown",
                                  facecolor="burlywood", label="Caja")
        ax.add_patch(rect)

    # Simulación rápida (hacia cada objetivo)
    current_target_idx = 0
    for _ in range(800):
        if current_target_idx >= len(targets):
            break
        tx, ty = targets[current_target_idx]
        vl, vr = robot.compute_control(tx, ty, boxes)
        robot.skid_steer_model(vl, vr)
        if robot.boxes_pushed > current_target_idx:
            current_target_idx += 1

    # Trazar trayectoria
    ax.plot(robot.traj_x, robot.traj_y, "b-", linewidth=1.5, label="Trayectoria Husky")
    ax.plot(robot.traj_x[0], robot.traj_y[0], "go", markersize=8, label="Inicio")
    ax.plot(robot.traj_x[-1], robot.traj_y[-1], "rs", markersize=8, label="Fin")

    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, linestyle="--", alpha=0.4)
    plt.tight_layout()
    plt.savefig("husky_sim_demo.png", dpi=120)
    plt.show()

    print("\n=== Estado final del Husky ===")
    print(robot.get_pose())


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    run_demo()
