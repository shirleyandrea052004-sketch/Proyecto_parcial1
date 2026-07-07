# fdg_sin_obstaculos_pkg — Piloto Autónomo Follow The Gap (F1TENTH)

Nodo ROS 2 que implementa el algoritmo Follow The Gap (FTG) con control PD, slew-rate, histéresis de selección de gap y ventana de visión dinámica, probado con éxito (10 vueltas sin colisiones) en la pista de Oschersleben dentro del simulador F1TENTH.

## Tabla de contenidos
- [Requisitos](#requisitos)
- [Descarga e instalación](#descarga-e-instalación)
- [Cómo ejecutar el nodo](#cómo-ejecutar-el-nodo)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Qué hace el algoritmo, paso a paso](#qué-hace-el-algoritmo-paso-a-paso)
- [Parámetros ajustables](#parámetros-ajustables)

## Requisitos
* Ubuntu 22.04 (o compatible)
* ROS 2 (Humble o Foxy)
* `f1tenth_gym_ros` y `f1tenth_gym` instalados por separado en tu workspace (este repositorio no incluye el simulador ni los mapas por defecto, solo el paquete del piloto).
* Python 3.10+
* Dependencias Python: `numpy` (`pip install numpy --break-system-packages`)

## Descarga e instalación
Este paquete debe ubicarse dentro de la carpeta `src/` de un workspace de ROS 2 junto al simulador `f1tenth_gym_ros` y el repositorio de mapas `f1tenth_racetracks`. Es obligatorio que la carpeta de mapas esté dentro de `src/` para que el simulador localice la pista.

**1. Crear (o reutilizar) el workspace**
```bash
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src
```

**2. Clonar este repositorio y dependencias necesarias dentro de `src/`**
```bash
# Clonar este repositorio y extraer el paquete del piloto
git clone [https://github.com/shirleyandrea052004-sketch/Proyecto_parcial1.git](https://github.com/shirleyandrea052004-sketch/Proyecto_parcial1.git)
mv Proyecto_parcial1/fdg_sin_obstaculos_pkg .
rm -rf Proyecto_parcial1

# Clonar el simulador oficial y la carpeta de mapas f1tenth_racetracks
git clone [https://github.com/f1tenth/f1tenth_gym_ros.git](https://github.com/f1tenth/f1tenth_gym_ros.git)
git clone [https://github.com/f1tenth/f1tenth_racetracks.git](https://github.com/f1tenth/f1tenth_racetracks.git)
```

**3. Compilar el entorno**
```bash
cd ~/f1tenth_ws
colcon build --packages-select fdg_sin_obstaculos_pkg f1tenth_gym_ros
```

**4. Configurar el entorno**
```bash
source ~/f1tenth_ws/install/setup.bash
```

## Cómo ejecutar el nodo

### Configuración Previa del Simulador
Antes de lanzar el entorno, es necesario configurar la ruta absoluta del mapa en el simulador para que coincida con la ubicación en tu sistema local.
1. Abre el archivo de configuración: `nano ~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml`
2. Modifica la variable `map_path` reemplazando `TU_USUARIO` con tu nombre de usuario en Ubuntu para apuntar al mapa de Oschersleben dentro de `f1tenth_racetracks`:
   `map_path: '/home/TU_USUARIO/f1tenth_ws/src/f1tenth_racetracks/Oschersleben/Oschersleben_map'`

**1. Levanta el simulador F1TENTH**
En una terminal (con el entorno sourceado): `source install/setup.bash`
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**2. Corre el nodo del piloto FTG**
En otra terminal (también sourceada):
```bash
ros2 run fdg_sin_obstaculos_pkg follow_the_gap
```

Deberías ver en consola mensajes confirmando la conexión a la odometría y, cada vez que complete una vuelta, el tiempo registrado.

## Estructura del paquete
```text
fdg_sin_obstaculos_pkg/
├── fdg_sin_obstaculos_pkg/
│   ├── __init__.py
│   └── follow_the_gap_node.py     # Nodo principal
├── resource/
├── test/
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Qué hace el algoritmo
* **Paso 0 — Preprocesamiento del LiDAR:** Los valores `inf`/`nan` se rellenan con la última lectura válida del mismo rayo para evitar dropouts. Se aplica un suavizado por convolución (media móvil, ventana 5).
* **Paso 1 — Burbuja de seguridad:** Se identifica el punto más cercano y se "borra" una franja angular a su alrededor del ancho del auto.
* **Paso 2 — Ventana de visión dinámica:** El FOV se ajusta según la velocidad (más angosto en rectas, amplio en curvas). Se agrupan lecturas válidas en "gaps" y se elige el más largo aplicando histéresis para evitar saltos bruscos.
* **Paso 3 — Trazada dentro del gap:** El punto objetivo mezcla el punto más profundo del gap y el centro del mismo, ajustándose según la estabilidad de la escena (EMA).
* **Paso 4 — Filtro EMA:** El ángulo objetivo se suaviza con una media móvil exponencial antes del controlador.
* **Paso 5 — Control PD + slew-rate:** Un controlador calcula la dirección, limitada por un slew-rate (cambios máximos permitidos) y un deadband que ignora correcciones minúsculas.
* **Paso 6 — Acelerador:** La velocidad se interpola dinámicamente dependiendo del ángulo de giro actual y la distancia libre frontal.

## Parámetros ajustables

| Parámetro | Rol | Valor actual |
| :--- | :--- | :--- |
| `car_width` | Ancho usado para la burbuja de seguridad | 1.67 |
| `hard_min_clearance` | Distancia mínima considerada "camino válido" | 0.55 m |
| `bubble_min_dist` | Piso de distancia para el cálculo de la burbuja | 0.28 m |
| `max_weight_depth` | Máximo peso hacia el punto más profundo del gap | 0.60 |
| `ema_alpha` | Suavizado del ángulo objetivo | 0.45 |
| `Kp / Kd` | Ganancias del controlador PD | 0.50 / 0.46 |
| `rate_open / rate_tight` | Límite de cambio de dirección por ciclo | 0.05 / 0.09 rad |
| `angle_trigger_min / _max`| Umbrales que gradúan el slew-rate | 0.07 / 0.27 rad |
| `steering_deadband` | Umbral mínimo de corrección aplicada | 0.015 rad |
| `speed_low / speed_high` | Rango de velocidad para interpolar el FOV dinámico | 4.5 / 9.0 m/s |
| `gap_switch_margin` | Cuánto debe superar un gap nuevo al anterior para reemplazarlo | 1.20 (20%) |
```