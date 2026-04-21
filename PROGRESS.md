# 📊 Dashboard de Progreso - ANYmal Robot

**Rama**: `anymal`  
**Última actualización**: Abril 15, 2026

---

## 📈 Resumen del Equipo

| Métrica | Valor | Estado |
|---------|-------|--------|
| Progreso General | 0% | 🔴 No iniciado |
| Tareas Completadas | 0/10 | 🔴 Pendientes |
| Bugs Reportados | 0 | ✅ Sin issues |
| Documentación | Plantilla | ⚠️ En desarrollo |

---

## 🎯 Hitos del Sprint

- [ ] Configuración del entorno
- [ ] Implementación de controladores básicos
- [ ] Integración de sensores
- [ ] Pruebas en simulación Gazebo
- [ ] Optimización de locomoción
- [ ] Primera integración con Tech Lead

---

## 📋 Tareas del Equipo

| # | Tarea | Responsable | Estado | Prioridad | Fecha Límite |
|----|-------|-------------|--------|-----------|--------------|
| 1 | Setup ambiente ROS/ROS2 | - | 🔴 No iniciado | Alta | Abril 18 |
| 2 | Implementar controlador base | - | 🔴 No iniciado | Alta | Abril 22 |
| 3 | Integración LiDAR | - | 🔴 No iniciado | Media | Mayo 1 |
| 4 | Integración cámara | - | 🔴 No iniciado | Media | Mayo 5 |
| 5 | Pruebas de locomoción | - | 🔴 No iniciado | Alta | Mayo 8 |
| 6 | Navegación autónoma | - | 🔴 No iniciado | Media | Mayo 15 |
| 7 | Documentación técnica | - | 🔴 No iniciado | Media | Mayo 20 |
| 8 | Pruebas de performance | - | 🔴 No iniciado | Media | Mayo 25 |
| 9 | Integración final | - | 🔴 No iniciado | Alta | Junio 1 |
| 10 | Entrega y presentación | - | 🔴 No iniciado | Alta | Junio 8 |

---

## 🔧 Dependencias y Bloqueadores

- [ ] Esqueleto base validado por Tech Lead
- [ ] Entorno de simulación funcional
- [ ] Acceso a especificaciones de hardware

---

## 🧪 Pruebas Completadas

- [ ] Prueba de comunicación ROS
- [ ] Prueba de movimiento básico
- [ ] Prueba de sensores
- [ ] Prueba de navegación

---

## 📝 Notas del Equipo

**Pendiente**: Primer kickoff con el equipo

---

## 🔗 Referencias

- [README.md](README.md) - Descripción general
- Branch principal: `main`
- Tech Lead: Disponible en branch main

---

## 🤖 Desarrollo técnico - Parte ANYmal

Esta sección concentra la parte escrita y técnica del ANYmal para que quede junto con el dashboard del equipo.

---

## 1) Question 4 — Direct kinematics of one ANYmal leg

Given:

$$
l_0=0.0585,\quad l_1=0.35,\quad l_2=0.33,\quad s=+1
$$

$$
(q_1,q_2,q_3)=(0,0.7,-1.4)\text{ rad}
$$

Using the equations from the sheet:

$$
x=l_1\sin q_2+l_2\sin(q_2+q_3)
$$

$$
y=s\,l_0\cos q_1
$$

$$
z=-l_1\cos q_2-l_2\cos(q_2+q_3)
$$

Since:

$$
q_2+q_3=0.7-1.4=-0.7
$$

$$
\sin(0.7)\approx 0.6442,\quad \sin(-0.7)\approx -0.6442
$$

$$
\cos(0.7)\approx 0.7648,\quad \cos(-0.7)\approx 0.7648
$$

Then:

$$
x=0.35(0.6442)+0.33(-0.6442)\approx 0.0129\text{ m}
$$

$$
y=(+1)(0.0585)\cos(0)=0.0585\text{ m}
$$

$$
z=-0.35(0.7648)-0.33(0.7648)\approx -0.5201\text{ m}
$$

**Answer:**

$$
\boxed{p_{foot}\approx (0.0129,\;0.0585,\;-0.5201)\text{ m}}
$$

That is the foot position for that leg configuration.

---

## 2) Question 5 — Jacobian determinant and singularities

The Jacobian is:

$$
J(q)=
\begin{bmatrix}
0 & l_1\cos q_2+l_2\cos(q_2+q_3) & l_2\cos(q_2+q_3) \\
-s\,l_0\sin q_1 & 0 & 0 \\
0 & l_1\sin q_2+l_2\sin(q_2+q_3) & l_2\sin(q_2+q_3)
\end{bmatrix}
$$

Expanding by the first column:

$$
\det(J)=(-s\,l_0\sin q_1)(-1)\;
\det
\begin{bmatrix}
l_1\cos q_2+l_2\cos(q_2+q_3) & l_2\cos(q_2+q_3)\\
l_1\sin q_2+l_2\sin(q_2+q_3) & l_2\sin(q_2+q_3)
\end{bmatrix}
$$

After simplifying the \(2\times2\) determinant:

$$
\det(J)=s\,l_0\,l_1\,l_2\,\sin q_1\,\sin q_3
$$

So the final symbolic answer is:

$$
\boxed{\det(J)=s\,l_0\,l_1\,l_2\,\sin q_1\,\sin q_3}
$$

### Singularities

$$
\det(J)=0 \quad \Rightarrow \quad \sin q_1=0 \;\text{ or }\; \sin q_3=0
$$

So:

$$
\boxed{q_1=k\pi \quad \text{or} \quad q_3=k\pi,\; k\in\mathbb{Z}}
$$

### Geometric interpretation

- **\(q_1=k\pi\):** the first joint loses instantaneous influence in the lateral direction, so one motion direction is lost.
- **\(q_3=k\pi\):** the leg links become collinear, especially at \(q_3\to 0\), which is the classic stretched-leg singularity.

---

## 3) Question 6 — Force control with \( \tau = J^T f \)

The desired foot force is:

$$
f_{pie}=
\begin{bmatrix}
10\\
0\\
-50
\end{bmatrix}\text{ N}
$$

The equation is:

$$
\boxed{\tau = J^T f_{pie}}
$$

With

$$
J=
\begin{bmatrix}
0 & 0.45 & 0.23\\
0 & 0 & 0\\
0 & 0.38 & 0.19
\end{bmatrix}
$$

Then

$$
J^T=
\begin{bmatrix}
0 & 0 & 0\\
0.45 & 0 & 0.38\\
0.23 & 0 & 0.19
\end{bmatrix}
$$

Now multiply:

$$
\tau=
\begin{bmatrix}
0 & 0 & 0\\
0.45 & 0 & 0.38\\
0.23 & 0 & 0.19
\end{bmatrix}
\begin{bmatrix}
10\\
0\\
-50
\end{bmatrix}
$$

So:

$$
\tau_1=0
$$

$$
\tau_2=0.45(10)+0.38(-50)=4.5-19=-14.5
$$

$$
\tau_3=0.23(10)+0.19(-50)=2.3-9.5=-7.2
$$

**Answer:**

$$
\boxed{\tau=
\begin{bmatrix}
0\\
-14.5\\
-7.2
\end{bmatrix}\text{ N·m}}
$$

### Why is \( \tau_1=0 \)?

Because the first column of \(J\) is zero in this configuration, so joint \(q_1\) does not contribute to the foot motion in the force directions being applied.

### What if the force had a \(y\)-component?

In a general configuration, a \(y\)-component would usually couple into \(q_1\), so \( \tau_1 \) would no longer be zero. In this exact numeric Jacobian, though, the entire second row is zero, so even a nonzero \(f_y\) would still not produce torque through this specific matrix.

---

## 4) Question 7.1 — Complexity of ANYmal IK

The correct choice is:

$$
\boxed{O(1)}
$$

Because it is geometric, closed-form IK for one leg, so each solve is constant time.

---

## 5) Question 8 — Pseudocode for motion planning of one ANYmal leg

```text
INPUT: current joint state q_current, desired foot point p_d, number of samples N

1. Compute current foot position p_0 = FK(q_current)
2. For k = 0 to N:
3.     alpha = k / N
4.     p_k = (1 - alpha) * p_0 + alpha * p_d      // Cartesian interpolation
5.     q_k = IK(p_k)                              // closed-form geometric IK
6.     J_k = Jacobian(q_k)
7.     if |det(J_k)| <= 1e-3:
8.         modify p_k or stop trajectory          // avoid singularity
9.     store q_k
10. Smooth stored joint trajectory if needed
11. Send q_k sequence to gait/controller

OUTPUT: smooth joint trajectory from current pose to p_d
```

### Complexity

- Each FK, IK, and determinant check is \(O(1)\)
- Done for \(N\) samples

So total complexity is:

$$
\boxed{O(N)}
$$

### Assumptions

- IK has a closed-form solution
- Desired point is inside the reachable workspace
- Singular points are rejected or adjusted
- Controller can track the generated joint sequence smoothly

---

## 6) For the mini reto: what `anymal_gait.py` should do

For Phase 2, your ANYmal module should:

- carry the 3 PuzzleBots,
- walk in trot,
- plan each foot in Cartesian space,
- solve IK for each foot point,
- monitor \(|\det J|>10^{-3}\),
- and reach the destination with low final error.

A clean structure for `anymal_gait.py` is:

```python
import numpy as np

class AnymalLeg:
    def __init__(self, l0=0.0585, l1=0.35, l2=0.33, s=1):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.s = s

    def fk(self, q):
        q1, q2, q3 = q
        x = self.l1*np.sin(q2) + self.l2*np.sin(q2 + q3)
        y = self.s*self.l0*np.cos(q1)
        z = -self.l1*np.cos(q2) - self.l2*np.cos(q2 + q3)
        return np.array([x, y, z])

    def jacobian(self, q):
        q1, q2, q3 = q
        return np.array([
            [0,
             self.l1*np.cos(q2) + self.l2*np.cos(q2+q3),
             self.l2*np.cos(q2+q3)],
            [-self.s*self.l0*np.sin(q1), 0, 0],
            [0,
             self.l1*np.sin(q2) + self.l2*np.sin(q2+q3),
             self.l2*np.sin(q2+q3)]
        ])

    def det_jacobian(self, q):
        return np.linalg.det(self.jacobian(q))

    def ik(self, p):
        x, y, z = p

        q1 = np.arccos(np.clip(y / (self.s*self.l0), -1.0, 1.0))

        D = (x**2 + z**2 - self.l1**2 - self.l2**2) / (2*self.l1*self.l2)
        D = np.clip(D, -1.0, 1.0)
        q3 = np.arctan2(-np.sqrt(1 - D**2), D)   # bent-knee branch

        q2 = np.arctan2(x, -z) - np.arctan2(self.l2*np.sin(q3), self.l1 + self.l2*np.cos(q3))
        return np.array([q1, q2, q3])
```

Then in the gait generator:

1. define swing and stance foot trajectories,
2. interpolate each foot target,
3. solve IK point by point,
4. reject points where `abs(detJ) <= 1e-3`,
5. command the valid joint sequence.

That is enough for a solid first pass.

---

## ✅ Suggested next updates for this dashboard

- [ ] Assign responsables for `anymal_gait.py`, testing, and documentation
- [ ] Add current simulation status
- [ ] Add plots or logs for `det(J)` monitoring
- [ ] Track final position error to target
- [ ] Note singularity cases found during testing

---

**Instrucciones**: Actualizar este dashboard regularmente con el avance del equipo.
