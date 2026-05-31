Aquí tienes el archivo `README.md` completamente limpio y consolidado en un único bloque. Solo tienes que darle al botón de copiar en la esquina superior derecha del cuadro y pegarlo en tu archivo.

```markdown
# 🤖 Proyecto ABP - Exploración Multirobot en ROS2

## 📖 Descripción
Este proyecto implementa un sistema multi-robot desarrollado en **ROS2 Jazzy** utilizando el simulador **Gazebo Harmonic**. El sistema integra herramientas avanzadas como Nav2 y SLAM Toolbox para llevar a cabo tareas de exploración, mapeo, detección de objetivos mediante visión artificial (cámara) y navegación coordinada de forma autónoma.

## 📂 Estructura del Proyecto

```text
├── src/
│   └── proyecto_abp/          # Paquete principal de ROS2
│       ├── launch/            # Archivos de lanzamiento (.launch.py)
│       ├── config/            # Archivos de configuración (YAML y mapas)
|       ├── rviz/              # Archivos de configuración RViZ
│       ├── urdf/              # Modelos y descripciones de los robots (Xacro)
│       ├── world/             # Entornos de simulación en Gazebo (.sdf)
│       ├── proyecto_abp/      # Nodos de Python (Cámara, Láser, FSM, Controladores, SLAM, Nav)
|       ├── setup.cfg          # Archivo de config
|       ├── setup.py           # Entry point y ejecutables
|       ├── package.xml        # Dependencias para los archivos launch
|       ├── requirements.md            # Dependencias detalladas del proyecto
|       └── README.md                  # Este documento

```

---

## ⚙️ Requisitos y Dependencias

Para mantener este documento limpio, todas las dependencias de Python (librerías de visión y matemáticas) y los paquetes del sistema de ROS2 Jazzy (Nav2, Gazebo, SLAM) se han documentado por separado.

👉 **Por favor, consulta el archivo [requirements.md] para ver la lista completa de dependencias y las instrucciones detalladas de instalación antes de continuar.**

---

## 🚀 Compilación del Proyecto

Asegúrate de clonar este repositorio dentro de la carpeta `src` de tu espacio de trabajo (workspace) de ROS2.

```bash
# 1. Crear el workspace (si no tienes uno)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clonar el repositorio
git clone <URL_DE_TU_REPOSITORIO> proyecto_abp

# 3. Moverse a la raíz del workspace
cd ~/ros2_ws

```

---

## 🎮 Ejecución

Para la ejecución del sistema multirobot con **2 robots** y siendo el objetivo de la búsqueda el objeto de color **verde**, se necesitan cuatro terminales abiertos en paralelo.

La dirección de trabajo en cada terminal debe ser el **directorio raíz o *workspace*** donde esté instalado ROS2 Jazzy (ej. `~/ros2_ws`), teniendo el paquete `proyecto_abp` bajo la carpeta `src/`.

Es fundamental ejecutar los comandos en cada terminal siguiendo estrictamente el **orden enumerado** a continuación:

### 1. Terminal 1: Simulación y Entorno Base

Este terminal se encarga de compilar el paquete, cargar el entorno y lanzar RViz, Gazebo junto a nodos críticos como la máquina de estados finitos (FSM).

```bash
colcon build --packages-select proyecto_abp
source install/setup.bash
ros2 launch proyecto_abp main.launch.py

```

*(Nota: `main.launch.py` lanza por defecto 2 robots y busca el color verde. Si se desea cambiar, se pueden pasar parámetros como `num_robots:=3 goal:=blue`).*

### 2. Terminal 2: Lanzamiento de SLAM

Este terminal inicia las herramientas de mapeado y localización simultánea para ambos robots, integrando los datos de sus sensores.

```bash
source install/setup.bash
ros2 run proyecto_abp start_slam 2

```

### 3. Terminal 3: Lanzamiento de Nav2

Este terminal arranca el coordinador y los servidores de acción de Navigation2 para la navegación y planificación de trayectorias.

```bash
source install/setup.bash
ros2 run proyecto_abp start_nav 2

```

### 4. Terminal 4: Lógica de Control

Este terminal activa de manera orquestada el procesamiento de la cámara, el procesamiento láser y el controlador PD encargado de la evasión de obstáculos y seguimiento visual.

```bash
source install/setup.bash
ros2 run proyecto_abp start_logic 2

```

```

```
