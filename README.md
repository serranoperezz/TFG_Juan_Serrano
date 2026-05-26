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
├── ntrip_ros/                  # PAQUETE 1: Cliente NTRIP para la Red Andaluza de Posicionamiento (RAP)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src/
│       └── ntripclient.py      # Captura hilos asíncronos y decodifica tramas RTCM 3
│
└── tfg_juan_serrano/           # PAQUETE 2: Paquete principal de control y navegación
    ├── CMakeLists.txt
    ├── package.xml
    ├── tfg_juan_serrano.code-workspace
    │
    ├── config/                 # Archivos de parametrización (YAML)
    │   ├── ekf_fusion.yaml     # Configuración de matrices de covarianza y selección de variables del EKF
    │   └── zed-f9p.yaml        # Configuración del modelo dinámico automotriz y tasa de hercios del u-blox
    │
    ├── launch/                 # Orquestadores de lanzamiento (LAUNCH)
    │   ├── fusion_gps.launch   # Inicializa el Filtro de Kalman y el transformador de coordenadas UTM
    │   ├── ntrip_rap.launch    # Levanta el cliente de red conectado a los servidores de la RAP
    │   ├── gps_master.launch   # Launch maestro definitivo para pruebas físicas en el coche
    │   └── sim_master.launch   # Launch espejo adaptado para el simulador Gazebo
    │
    └── src/                    # Nodos lógicos de decisión (PYTHON)
        ├── cmd_vel_to_ackermann.py  # Traductor cinemático de comandos geométricos a VESC
        ├── controlador_estados.py   # Supervisor central (Máquina de Estados Finitos)
        ├── grabador_trayectoria.py  # Registro espacial diferencial de waypoints (Notario)
        └── seguidor_puntos.py       # Algoritmo de guiado Pure-Pursuit y velocidad adaptativa (Piloto)
```

---

## 🛠️ Requisitos y Dependencias

El software ha sido validado en la plataforma física del vehículo (**NVIDIA Jetson TX2**) bajo el siguiente entorno:
* **S.O.:** Ubuntu 18.04 LTS
* **Meta-Sistema Operativo:** ROS Melodic Morenia
* **Dependencias de ROS:**
  ```bash
  sudo apt-get install ros-melodic-robot-localization ros-melodic-hector-trajectory-server ros-melodic-rtcm-msgs xterm
  ```

---

## 🚀 Instalación y Compilación

Para despliegue en un espacio de trabajo local de ROS, ejecuta los siguientes comandos en tu terminal de Linux:

```bash
# 1. Crear y acceder al espacio de trabajo
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. Clonar el repositorio
git clone https://github.com/serranoperezz/TFG_JuanSerrano.git .

# 3. Compilar el espacio de trabajo
cd ~/catkin_ws
catkin_make

# 4. Cargar las variables de entorno
source devel/setup.bash
```

---

## 🕹️ Modos de Uso (Flujo de Trabajo en Pista)

El sistema cuenta con un archivo de lanzamiento maestro unificado que automatiza la carga de todos los componentes de hardware e hilos lógicos.

### Lanzamiento de la Plataforma Real
Asegúrate de que la placa ArduSimple esté conectada en el puerto `/dev/ttyUSB0` y ejecuta:
```bash
roslaunch tfg_juan_serrano gps_master.launch
```
*Nota: Este comando desplegará automáticamente una ventana de terminal `xterm` independiente con el menú interactivo del **Director de Estados**.*

### Operación en Pista (Flujo Técnico)

1. **Fase de Reposo (IDLE):** El coche arranca bloqueado por seguridad. Las transformaciones geométricas (`tf`) se estabilizan con los datos fijos del RTK centimétrico de la RAP.
2. **Fase de Grabación (Mapeo):** * Pulsa el botón **`SELECT`** en el mando de PS3 o introduce la tecla **`G`** en la terminal de xterm.
   * Conduce el coche manualmente por la trazada ideal de la pista. El sistema guardará un punto de posición corregido de forma inteligente cada 5 cm en `~/trayectoria.csv`.
   * Vuelve a pulsar **`SELECT`** / **`G`** para cerrar y salvar físicamente el archivo en el eMMC.
3. **Fase de Carrera Autónoma (Navegación):**
   * Sitúa el vehículo en el punto de inicio de la trazada.
   * Pulsa el botón **`START`** en el mando o introduce la tecla **`R`** en la terminal.
   * El coche ejecutará de forma autónoma la trazada calculando el error de rumbo y atenuando su velocidad lineal en las curvas cerradas.
4. **Interrupción de Emergencia:** Pulsa **`FLECHA ABAJO`** en el mando para abortar instantáneamente el modo autónomo y forzar el frenado absoluto del motor.

---

## 📄 Licencia

Este proyecto está bajo la **Licencia MIT** - consulta el archivo de licencias para más detalles. Tu uso de este código es libre bajo reconocimiento de la autoría original.
