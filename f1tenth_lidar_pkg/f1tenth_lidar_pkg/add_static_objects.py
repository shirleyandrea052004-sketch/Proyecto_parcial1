#!/usr/bin/env python3
"""
add_static_obstacles.py
========================
Agrega obstáculos estáticos a un mapa de F1Tenth (par .png + .yaml) pintando
círculos "ocupados" directamente sobre la imagen del mapa. El simulador
f1tenth_gym hace ray-casting de LiDAR contra esa imagen, así que un círculo
negro se convierte en un obstáculo real y sólido que tu auto debe esquivar.

POR QUÉ ESTE MÉTODO:
El bridge ROS de este simulador (f1tenth_gym_ros) no expone una API para
"agregar objetos" en tiempo de ejecución — el mundo físico que ve el LiDAR
ES la imagen del mapa. Por eso, para el punto 3 de tu rúbrica (5 obstáculos
estáticos), la forma estándar es generar una variante del mapa con esos
obstáculos ya "pintados" y apuntar sim.yaml a ese nuevo mapa.

CÓMO ENCONTRAR LAS COORDENADAS (x, y) EN METROS PARA TUS 5 OBSTÁCULOS:
1. Lanza el simulador normal: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
2. Abre RViz (se abre junto con el simulador) y usa la herramienta
   "Publish Point" (o simplemente pasa el mouse sobre el mapa: RViz muestra
   las coordenadas (x, y) en la barra inferior izquierda).
3. Anota 5 puntos sobre la pista, en lugares donde el auto tenga que
   esquivarlos pero sin bloquear el camino por completo.

USO:
    python3 add_static_obstacles.py \
        --map levine.yaml \
        --output levine_obstacles \
        --obstacle 5.0 2.0 0.25 \
        --obstacle -3.2 1.0 0.25 \
        --obstacle 0.0 -4.5 0.3 \
        --obstacle 8.0 0.0 0.25 \
        --obstacle -6.0 -2.0 0.3

Cada --obstacle recibe: x_metros y_metros radio_metros

Esto genera levine_obstacles.png y levine_obstacles.yaml en la carpeta de
salida. Copia ambos a tu carpeta maps/ del paquete f1tenth_gym_ros y apunta
map_path en sim.yaml a este nuevo mapa (sin la extensión).
"""
import argparse
import os
import yaml
from PIL import Image, ImageDraw


def world_to_pixel(x, y, origin, resolution, img_height):
    """
    Convierte coordenadas del mundo (metros) a coordenadas de pixel.
    El origen del mapa (origin) es la esquina INFERIOR IZQUIERDA en el
    sistema de coordenadas del mundo, pero en la imagen el pixel (0,0)
    es la esquina SUPERIOR IZQUIERDA -> por eso se invierte el eje Y.
    """
    px = (x - origin[0]) / resolution
    py = img_height - (y - origin[1]) / resolution
    return int(px), int(py)


def main():
    parser = argparse.ArgumentParser(description="Agrega obstáculos estáticos a un mapa F1Tenth")
    parser.add_argument('--map', required=True, help="Ruta al .yaml del mapa original (ej: levine.yaml)")
    parser.add_argument('--output', required=True, help="Nombre base de salida, sin extensión (ej: levine_obstacles)")
    parser.add_argument('--obstacle', nargs=3, type=float, action='append', metavar=('X', 'Y', 'RADIO'),
                         help="Obstáculo: x y radio (metros). Repetir una vez por obstáculo.")
    args = parser.parse_args()

    if not args.obstacle:
        print("⚠️  No se especificó ningún --obstacle. No hay nada que hacer.")
        return

    map_dir = os.path.dirname(os.path.abspath(args.map))
    with open(args.map, 'r') as f:
        map_yaml = yaml.safe_load(f)

    image_path = os.path.join(map_dir, map_yaml['image'])
    resolution = map_yaml['resolution']
    origin = map_yaml['origin']

    img = Image.open(image_path).convert('L')  # escala de grises, igual que el original
    width, height = img.size
    draw = ImageDraw.Draw(img)

    print(f"Mapa cargado: {image_path} ({width}x{height} px, resolución {resolution} m/px)")

    for (x, y, radius_m) in args.obstacle:
        px, py = world_to_pixel(x, y, origin, resolution, height)
        radius_px = max(1, int(radius_m / resolution))
        # Pintamos negro (0) = ocupado, igual que las paredes del mapa.
        draw.ellipse(
            [px - radius_px, py - radius_px, px + radius_px, py + radius_px],
            fill=0
        )
        print(f"  Obstáculo en mundo=({x:.2f}, {y:.2f}) m, radio={radius_m:.2f} m "
              f"-> pixel=({px}, {py}), radio_px={radius_px}")

    out_dir = os.path.dirname(os.path.abspath(args.output)) or '.'
    out_name = os.path.basename(args.output)
    os.makedirs(out_dir, exist_ok=True)

    out_png = os.path.join(out_dir, out_name + '.png')
    out_yaml_path = os.path.join(out_dir, out_name + '.yaml')

    img.save(out_png)

    map_yaml['image'] = out_name + '.png'
    with open(out_yaml_path, 'w') as f:
        yaml.dump(map_yaml, f, default_flow_style=None)

    print(f"\n✅ Listo. Generado:\n  {out_png}\n  {out_yaml_path}")
    print(f"\nSiguiente paso: copia ambos archivos a la carpeta maps/ de tu paquete "
          f"f1tenth_gym_ros, y en sim.yaml apunta:\n"
          f"  map_path: '/home/tu_usuario/F1Tenth-Repository/src/f1tenth_gym_ros/maps/{out_name}'")


if __name__ == '__main__':
    main()