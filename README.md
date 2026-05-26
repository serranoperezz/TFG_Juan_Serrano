# TFG: Navegación Autónoma y Localización Centimétrica (UMA-RACECAR)

Este repositorio contiene los paquetes de ROS desarrollados para el Trabajo Fin de Grado: **"Integración de IMU y GNSS-RTK para seguimiento de
trayectorias con vehículo a escala para la competición de Formula Student"** en la Escuela de Ingenierías Industriales de la Universidad de Málaga (UMA).

El sistema utiliza una arquitectura modular que combina fusión sensorial mediante Filtro de Kalman Extendido (EKF), correcciones diferenciales en tiempo real vía protocolo NTRIP y un algoritmo de guiado geométrico por persecución pura (*Pure-Pursuit*).

## 📊 Arquitectura del Software

El ecosistema se divide en tres capas lógicas de ejecución:
1. **Sensado (Inputs):** Captura de datos brutos del receptor u-blox ZED-F9P, la IMU y la odometría base del controlador VESC.
2. **Fusión Sensorial:** Ejecución de `robot_localization` para estabilizar la pose filtrada en coordenadas cartesianas locales (marco `map`).
3. **Decisión y Guiado:** Control central mediante una máquina de estados que gestiona la grabación de la pista y su posterior reproducción autónoma.

---

## 📂 Estructura del Repositorio

```text
├── ntrip_ros/                # Cliente NTRIP para la Red Andaluza de Posicionamiento (RAP)
│   └── src/ntripclient.py    # Captura hilos asíncronos y decodifica tramas RTCM 3
└── tfg_juan_serrano/         # Paquete principal de control y navegación
    ├── config/               # Parametrización del EKF y del chip u-blox (YAML)
    ├── launch/               # Orquestadores de simulación y entorno real (LAUNCH)
    └── src/                  # Nodos lógicos de decisión (Python)
        ├── controlador_estados.py  # Supervisor central (Máquina de Estados)
        ├── grabador_trayectoria.py # Registro espacial diferencial de waypoints
        └── seguidor_puntos.py      # Algoritmo de guiado y velocidad adaptativa
