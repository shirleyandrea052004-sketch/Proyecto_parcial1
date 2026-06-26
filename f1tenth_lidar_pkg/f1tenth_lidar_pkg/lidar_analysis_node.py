#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class LidarAnalysisNode(Node):
    def __init__(self):
        super().__init__('lidar_analysis_node')
        # Suscripción al topic del LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.last_time = None
        self.get_logger().info("Nodo inicializado. Esperando datos del LiDAR...")

    def scan_callback(self, msg):
        current_time = time.time()
        freq = 0.0
        
        # 1. Cálculo de frecuencia de publicación (2 pts)
        if self.last_time is not None:
            delta_time = current_time - self.last_time
            if delta_time > 0:
                freq = 1.0 / delta_time
        self.last_time = current_time

        # Esperar a la segunda iteración para tener una diferencia de tiempo válida
        if freq == 0.0:
            return 

        # 2. Cálculo de resolución angular en grados (2 pts)
        res_deg = math.degrees(msg.angle_increment)

        # 3. División de arreglos: frontales y traseras (1 pt)
        front_ranges = []
        rear_ranges = []

        for i, r in enumerate(msg.ranges):
            # Calcular el ángulo en radianes para la medición actual
            angle_rad = msg.angle_min + i * msg.angle_increment
            # Convertir a grados
            angle_deg = math.degrees(angle_rad)
            
            # Normalizar el ángulo a un rango de 0 a 360 grados
            angle_normalized = angle_deg % 360

            # Dividir según los criterios solicitados
            if 0 <= angle_normalized < 180:
                front_ranges.append(r)
            else:
                rear_ranges.append(r)

        # 4. Impresión por consola con el formato esperado
        print(f"Resolución angular del LiDAR: {res_deg:.3f} grados")
        print(f"Frecuencia del LiDAR: {freq:.3f} Hz")
        print(f"Cantidad de mediciones frontales (0° a 180°): {len(front_ranges)}")
        print(f"Cantidad de mediciones traseras (180° a 360°): {len(rear_ranges)}")
        
        # Función auxiliar para formatear los primeros 3 valores
        def format_sample(lst):
            # Filtramos valores infinitos para que se vea bien en consola si es necesario
            valid_lst = [x for x in lst if not math.isinf(x)]
            if len(valid_lst) >= 3:
                return f"[{valid_lst[0]:.2f}, {valid_lst[1]:.2f}, {valid_lst[2]:.2f}, ...]"
            return str([round(x, 2) for x in valid_lst])

        print(f"Primeros valores frontales: {format_sample(front_ranges)}")
        print(f"Primeros valores traseros: {format_sample(rear_ranges)}")
        print("-" * 50)
        
        # Opcional: Si el diseño requiere que finalice automáticamente tras la primera lectura, 
        # descomenta la siguiente línea. Si prefieres que siga iterando, déjala comentada.
        # raise SystemExit 

def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalysisNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass # Salida limpia si se utiliza raise SystemExit
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
