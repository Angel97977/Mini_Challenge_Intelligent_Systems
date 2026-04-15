# � Xarm - Brazo Robótico

**Branch**: `xarm`

## 📝 Descripción del Proyecto

Desarrollo de sistema de control para brazo robótico Xarm con capacidad de manipulación, control de trayectorias y integración de herramientas especializadas.

## 🎯 Objetivos del Equipo

- Implementar cinemática directa e inversa
- Desarrollar planificador de trayectorias
- Control de velocidad y fuerza
- Integración de gripper/actuadores
- Sistema de visión para cálculo de objetivos

## 📦 Estructura del Proyecto

```
src/
├── kinematics/          # Cinemática del brazo
├── control/             # Controladores PID
├── trajectory/          # Planificación de trayectorias
├── gripper/             # Control de gripper/herramientas
├── vision/              # Procesamiento de imagen
├── utils/               # Utilidades
└── docs/                # Documentación técnica

tests/
├── unit/                # Pruebas unitarias
├── kinematics_test/     # Validación de cinemática
├── trajectory_test/     # Pruebas de trayectorias
└── integration/         # Pruebas de integración
```

## 🚀 Primeros Pasos

1. Revisar `PROGRESS.md` para tareas asignadas
2. Revisar especificaciones de Xarm
3. Crear features en sub-branches: `feature/[nombre]`
4. Pushar cambios regularmente

## 🔧 Stack Tecnológico

- **Lenguaje**: Python 3.8+, C++
- **Framework**: ROS / ROS2
- **Control**: OpenCV, Numpy
- **Hardware**: Xarm 6 / Xarm 7
- **Versionado**: Git

## 📋 Checklist Inicial

- [ ] Entender especificaciones de Xarm
- [ ] Revisar documentación de cinemática
- [ ] Configurar entorno local
- [ ] Ejecutar pruebas iniciales
- [ ] Primer commit

## 👥 Equipo

- **Tech Lead**: -
- **Desarrolladores**: -

---

**Creado**: Abril 2026
