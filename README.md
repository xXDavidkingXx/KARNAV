# KARNAV — Kinetic Autonomous Robotic Navigation and Avoidance Vehicle (ROS 2 Humble)

**Autor:** Edwin David Sánchez Medina
**Proyecto:** Investigación en robótica industrial (navegación autónoma indoor sin encoders)
**Entorno:** Ubuntu 22.04 (Jammy) • ROS 2 Humble • Raspberry Pi 4B • LiDAR FHL‑LD20 • IMU AltIMU‑10 v5/MPU6050 • ESP32 (micro‑ROS)

> **Resumen:** KARNAV es un prototipo de robot móvil autónomo para navegación en interiores con detección y evasión de obstáculos. A diferencia de los enfoques clásicos, **no usa encoders**; la odometría se estima combinando **LiDAR** e **IMU** (sensor fusion), integrando *robot_localization*, *slam_toolbox* y *Nav2* en ROS 2 Humble.

---

## Tabla de contenidos

* [Características](#características)
* [Hardware](#hardware)
* [Software y dependencias](#software-y-dependencias)
* [Estructura del repositorio](#estructura-del-repositorio)
* [Instalación rápida](#instalación-rápida)
* [Compilación](#compilación)
* [Arranque rápido (bringup)](#arranque-rápido-bringup)
* [SLAM en línea](#slam-en-línea)
* [Navegación con Nav2](#navegación-con-nav2)
* [micro‑ROS (ESP32)](#micro‑ros-esp32)
* [Buenas prácticas](#buenas-prácticas)
* [Solución de problemas](#solución-de-problemas)
* [CI/CD](#cicd)
* [Licencia](#licencia)
* [Citación](#citación)
* [Créditos](#créditos)

---

## Características

* ✅ Navegación indoor autónoma con **detección y evasión de obstáculos**.
* ✅ **Sin encoders**: odometría basada en LiDAR + IMU (*robot_localization*).
* ✅ **SLAM** con *slam_toolbox* (modo síncrono y asíncrono).
* ✅ **Planeación y control** con *Nav2*.
* ✅ Arquitectura **multimachine** (Raspberry Pi + laptop) sin *domain_id* fijo.
* ✅ Soporte **micro‑ROS** para control bajo ESP32.

---

## Hardware

* Raspberry Pi 4B (4 GB RAM) — Ubuntu Server 22.04
* LiDAR Youyeetoo **FHL‑LD20** (driver `ldlidar` + Linorobot lib)
* IMU **AltIMU‑10 v5** (o **MPU6050** con micro‑ROS)
* **ESP32‑WROOM‑32** (motores/sensores)
* 4 ruedas **mecanum** + 4 motores TT (3–6 V)
* 2 Puentes H **L298N**
* Baterías: packs 3S1P + LiPo 7.4 V

> **Nota:** Verifica consumo de motores TT; limita picos para proteger L298N.

---

## Software y dependencias

* **Ubuntu 22.04.3 LTS (Jammy)**
* **ROS 2 Humble Hawksbill** (instalación `deb`)
* `colcon` + extensiones
* `rosdep`, `vcstool`
* Paquetes ROS: `robot_localization`, `slam_toolbox`, `nav2`, `rviz2`
* (Opcional) **ESP‑IDF 5.2** para códigos nativos y **micro‑ROS** (tutorial base en ESP‑IDF v4.1)

Instala requisitos mínimos:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-vcstool
sudo apt install -y ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-navigation2
```

---

## Estructura del repositorio

```
karnav/
├─ README.md
├─ LICENSE
├─ .gitignore
├─ .gitattributes                # Git LFS para meshes/mapas/binarios
├─ scripts/
│  ├─ setup_ubuntu_2204.sh       # instalación rápida en limpio
│  ├─ build_ros2.sh              # build + rosdep
│  └─ run_rviz_slam.sh           # lanza slam_toolbox
├─ docs/
│  ├─ hardware_wiring.md         # cableado, diagramas
│  └─ bringup_checklist.md       # checklist de arranque
├─ docker/
│  ├─ Dockerfile
│  └─ devcontainer.json
├─ .github/workflows/
│  └─ ci-colcon.yml              # CI: build y tests
├─ ros2_ws/
│  ├─ src/
│  │  ├─ karnav_description/     # URDF/meshes/launch
│  │  ├─ karnav_bringup/         # bringup y configs
│  │  ├─ karnav_slam_nav2/       # launch de slam + nav2
│  │  ├─ sensor_fusion/          # YAML robot_localization
│  │  ├─ ldlidar/                # driver (submódulo o propio)
│  │  └─ my_imu_launcher/
│  └─ ros2.repos                 # referencias externas (opcional)
└─ microros_ws/                  # opcional: firmware ESP32
   └─ firmware/esp32/
      ├─ CMakeLists.txt
      ├─ main/
      ├─ sdkconfig.defaults
      └─ components/
```

---

## Instalación rápida

Clona el repo y (opcional) importa fuentes externas con `vcstool`:

```bash
git clone <URL-del-repo> karnav
cd karnav/ros2_ws
# Si usas ros2.repos:
vcs import src < ros2.repos || true
```

Instala dependencias vía `rosdep`:

```bash
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y
```

---

## Compilación

```bash
cd karnav/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> Para sesiones nuevas, vuelve a `source install/setup.bash`.

---

## Arranque rápido (bringup)

1. Conecta LiDAR e IMU (revisa puertos y permisos udev).
2. Exporta ROS_DOMAIN_ID si lo requieres (por defecto sin domain):

   ```bash
   export ROS_DOMAIN_ID=0
   ```
3. Lanza el bringup del robot (ejemplo):

   ```bash
   ros2 launch karnav_bringup base_bringup.launch.py
   ```
4. Visualiza en RViz2:

   ```bash
   rviz2
   ```

   * *Fixed Frame*: `base_footprint` o `map` según etapa.
   * Activa **LaserScan**, **TF**, **RobotModel**.

---

## SLAM en línea

**slam_toolbox** (modo síncrono sugerido para navegación):

```bash
source ~/karnav/ros2_ws/install/setup.bash
ros2 launch karnav_slam_nav2 slam_sync.launch.py
```

Tips:

* Asegura `tf` consistentes: `odom -> base_footprint -> base_link -> sensores`.
* Si el láser parece rotado, verifica la estática `base_link -> ldlidar_base` y el eje X hacia **frente**, Y **izquierda**, Z **arriba**.
* Guarda mapa si deseas navegación con mapa estático:

  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/maps/karnav_map
  ```

---

## Navegación con Nav2

1. Inicia Nav2 (tras SLAM o con mapa cargado):

   ```bash
   ros2 launch karnav_slam_nav2 nav2_bringup.launch.py use_slam:=true
   ```
2. En RViz2, define **2D Pose Estimate** (si usas mapa estático) y **2D Goal Pose**.
3. Ajusta planners y controladores en los YAML de `karnav_slam_nav2/config/`.

> **Sin encoders:** `robot_localization` debe fusionar **IMU** (orientación/gyro) y **odometría lidar** (si la publicas como twist/pose) para estabilizar la pose.

---

## micro‑ROS (ESP32)

* Estrategia dual (por compatibilidad de tutoriales):

  * **ESP‑IDF 5.2** para tus proyectos nativos con `idf.py`.
  * **micro‑ROS** base (tutorial) en **ESP‑IDF v4.1**.
* Build/flash típico:

  ```bash
  cd microros_ws/firmware/esp32
  idf.py set-target esp32
  idf.py build
  idf.py flash monitor
  ```
* Publica tópicos de motores/sensores y conecta al **agente micro‑ROS** (host/puerto documentados en `README` local del firmware).

---

## Buenas prácticas

* Mantén **un workspace principal** (`ros2_ws`); usa `microros_ws` sólo si trabajas micro‑ROS.
* Evita conexiones sueltas en IMU/LiDAR: las **vibraciones** degradan la odometría.
* **Git LFS** para meshes (`*.stl`, `*.dae`, `*.obj`), imágenes y mapas (`*.pgm`, `*.yaml`), binarios (`*.bin`, `*.elf`).
* No subas **credenciales** (Wi‑Fi/SSH/tokens). Usa `secrets/` (ignorada por git).

---

## Solución de problemas

* **No veo el robot en RViz2**: revisa que `karnav_description` esté instalado y que `RobotModel` use el `TF` correcto (`base_link`).
* **Ejes invertidos/rotados**: verifica frames en RViz2 con `TF` (X al frente, Y izquierda, Z arriba). Ajusta estáticas en URDF.
* **`odom` separado en RViz2**: es normal si el `map->odom` se publica después; confirma transformaciones en `robot_localization`.
* **Picos de corriente** en motores TT: limita PWM/arranques, agrega capacitores y revisa temperatura de L298N.

---

## CI/CD

* *GitHub Actions* compila el workspace en Ubuntu 22.04 con ROS 2 Humble.
* Archivo: `.github/workflows/ci-colcon.yml`.
* Si usas `ros2.repos`, añade el paso `vcs import` en el job.

---

## Licencia

Este proyecto se distribuye bajo licencia **MIT** (ver `LICENSE`).

---

## Citación

Si usas este repositorio en investigación, por favor cita:

```
Sánchez Medina, E. D. (2025). KARNAV: Kinetic Autonomous Robotic Navigation and Avoidance Vehicle (ROS 2 Humble). Repositorio Git.
```

---

## Créditos

* ROS 2, slam_toolbox, Nav2, robot_localization, Linorobot LIDAR libs.
* Agradecimientos a la comunidad ROS por documentación y soporte.

---

> ¿Sugerencias? Abre un *issue* o envía un *pull request*. ¡Gracias!
