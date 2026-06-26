# F1Tenth - Controlador Reactivo Follow The Gap

Este repositorio contiene la implementación individual de un controlador puramente reactivo para el simulador F1Tenth, capaz de navegar pistas complejas y esquivar obstáculos dinámicos y estáticos a altas velocidades.

## Explicación del Funcionamiento del Código
El sistema de navegación autónoma opera mediante una versión avanzada del algoritmo Follow The Gap, ejecutando este ciclo continuo:

1. Preprocesamiento del LiDAR: Los datos del sensor pasan por un filtro de promedio móvil para suavizar el ruido e ignorar lecturas erróneas. Adicionalmente, el campo de visión se recorta a 180 grados frontales para evitar que el vehículo intente girar hacia atrás.
2. Burbuja de Seguridad Dinámica: El algoritmo localiza el obstáculo más cercano y crea una zona de exclusión (burbuja) a su alrededor inflando matemáticamente el ancho del vehículo. Esta expansión es adaptativa: crece cuando el obstáculo está más lejos para forzar un esquive anticipado y se contrae de cerca para pasar por espacios reducidos.
3. Identificación de Brechas: Se agrupan los rayos del escáner que están libres de obstáculos para identificar la brecha continua más amplia disponible en la pista.
4. Trazada Híbrida Estabilizada: Para decidir hacia dónde apuntar dentro del espacio libre, el algoritmo calcula un vértice dinámico asignando un 60% de prioridad al centro del hueco (evitando rozar las esquinas de los muros) y un 40% a la profundidad máxima (manteniendo al vehículo en línea recta durante tramos largos).
5. Controlador PD de Dirección: El ángulo objetivo se filtra a través de un Controlador Proporcional-Derivativo (PD) y un limitador de tasa de giro (Slew-Rate Limiter) adaptativo. Esto suprime las oscilaciones y garantiza que el servo de dirección responda con agilidad sin exceder los límites físicos del vehículo.
6. Velocidad Reactiva (Radar Frontal): La aceleración es dinámica. Un radar que evalúa los rayos centrales frontales reduce la velocidad a 4.5 m/s o 3.5 m/s si detecta un muro cercano o si el vehículo está ejecutando un giro agresivo. En rectas completamente despejadas, acelera a 6.0 m/s.

## Estructura del Código
* follow_the_gap_node.py: Nodo principal. Contiene el procesamiento del LiDAR, la lógica matemática del FTG, el controlador de la dirección, y el sistema de telemetría (cronómetro y contador de vueltas leyendo la odometría en /ego_racecar/odom).
* opponent_driver_node.py: Nodo auxiliar configurado para actuar como obstáculo dinámico en la pista, cumpliendo con las pruebas de entorno multi-agente.
* add_static_objects.py: Herramienta para pintar obstáculos estáticos en el mapa de la simulación.

## Instrucciones de Ejecución
Para reproducir la simulación completa con obstáculos dinámicos desde una sola terminal, ejecuta el siguiente script en la raíz de tu espacio de trabajo:

```bash
# 1. Cargar el entorno de ROS 2
source install/setup.bash

# 2. Lanzar el simulador y el mapa en segundo plano
ros2 launch f1tenth_gym_ros gym_bridge_launch.py &

# Esperar unos segundos a que el entorno cargue por completo
sleep 3

# 3. Lanzar el obstáculo dinámico en segundo plano
ros2 run f1tenth_lidar_pkg opponent_driver_node &

# 4. Lanzar el controlador principal Follow The Gap en primer plano
ros2 run f1tenth_lidar_pkg follow_the_gap_node
