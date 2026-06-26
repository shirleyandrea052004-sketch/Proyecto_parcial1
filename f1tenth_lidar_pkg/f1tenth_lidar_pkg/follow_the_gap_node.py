#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time
import math
from rclpy.qos import qos_profile_sensor_data


class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')

        # --- SUSCRIPTORES Y PUBLICADORES ---
        # Aceptamos el protocolo rápido del simulador!
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # --- PARÁMETROS DEL VEHÍCULO ---
        self.car_width = 0.50       # Margen de seguridad (burbuja) en metros
        self.safe_distance = 1.5    # Distancia para considerar frenar

        # --- VARIABLES DE TELEMETRÍA ---
        self.start_x = None
        self.start_y = None
        self.lap_start_time = None
        self.lap_count = 0
        self.distance_traveled = 0.0
        self.last_x = None
        self.last_y = None
        self.left_start_zone = False

        # ==========================================
        # ARÁMETROS DE ESTABILIZACIÓN ANTI-OSCILACIÓN
        # ==========================================

        # --- Capa 1: Filtro de entrada del LiDAR (Promedio Móvil) ---
        # Suaviza ruido rayo a rayo ANTES de buscar el hueco más profundo.
        # Window impar y pequeña: suficiente para matar ruido sin "ciegar" el sensor.
        self.lidar_filter_window = 3

        # --- Capa 2: Controlador PD sobre el ángulo de dirección ---
        # Kp: qué tan rápido persigue el ángulo objetivo (más alto = más reactivo, más oscilación)
        # Kd: amortiguamiento, frena el "rebote" anticipando la tendencia del error
        self.Kp = 0.45
        self.Kd = 0.40
        self.prev_error = 0.0

        # --- Capa 3: Limitador de Tasa de Giro ADAPTATIVO (Slew-Rate Limiter) ---
        # ANTES: un solo tope fijo para toda la pista. Eso fuerza un trade-off:
        # si lo bajas para eliminar oscilación en recta, también vuelves lento al
        # auto en las curvas técnicas (no llega a tiempo al ángulo real -> traza
        # ancho, frena tarde, pierde tiempo). En Oschersleben, con rectas largas
        # y curvas cerradas, ese trade-off se siente fuerte.
        # AHORA: dos topes, elegidos según qué tanto ángulo está pidiendo el FTG
        # (raw_steering_angle), NO según closest_dist. ¿Por qué el cambio?
        # closest_dist mide la pared más cercana en TODO el cono frontal, incluida
        # la pared lateral de una recta angosta -> eso disparaba falsos positivos
        # de "curva" en rectas, dejando el slew-rate "suelto" justo donde debía
        # quedarse firme en 0.03. El ángulo pedido no tiene ese problema: en una
        # recta real, raw_steering_angle ronda 0 sin importar qué tan cerca esté
        # la pared del costado.
        #   - rate_open  -> ángulo pedido pequeño (recta real): prioriza ESTABILIDAD
        #   - rate_tight -> ángulo pedido grande (curva real): prioriza AGILIDAD
        self.rate_open = 0.20
        self.rate_tight = 0.60
        self.angle_trigger_min = 0.07   # rad. Por debajo: se trata como "recta".
        self.angle_trigger_max = 0.27   # rad. Por encima: se trata como "curva cerrada".

        # --- Frenado de emergencia genuino (NO el mismo umbral que antes) ---
        # closest_dist < 1.4 disparaba en cualquier recta angosta con pared cerca
        # al costado, aunque el camino ADELANTE estuviera completamente libre.
        # Bajamos el umbral a algo que sí signifique "hay un obstáculo real cerca,
        # en cualquier dirección, frena ya" sin castigar rectas normales.
        self.emergency_dist = 0.5

        # --- Zona muerta (deadband) ---
        # Si el ángulo crudo es prácticamente cero, lo forzamos a cero.
        # Evita "vibrar" el volante por micro-ruido cuando el auto ya va recto.
        self.steering_deadband = 0.0

        self.prev_steering_angle = 0.0

        # --- Chequeo de salud de /ego_racecar/odom ---
        # Si el tópico de odometría estuviera mal (como pasó con '/odom' vs
        # '/ego_racecar/odom'), el contador de vueltas fallaría EN SILENCIO,
        # sin ningún error visible. Este timer avisa explícitamente si tras
        # 3 segundos no ha llegado ningún mensaje de odometría.
        self._odom_check_timer = self.create_timer(3.0, self._check_odom_connection)

        self.get_logger().info("Piloto Follow the Gap [v2 - Estabilizado] iniciado. Telemetría activada.")

    def _check_odom_connection(self):
        if self.start_x is None:
            self.get_logger().error(
                "⚠️  No se ha recibido NINGÚN mensaje en '/ego_racecar/odom' tras 3s. "
                "El contador de vueltas y el cronómetro NO van a funcionar. "
                "Verifica con 'ros2 topic list' y 'ros2 topic info /ego_racecar/odom' "
                "que el tópico exista y el nombre coincida exactamente."
            )
        else:
            self.get_logger().info("✅ Odometría conectada correctamente en '/ego_racecar/odom'.")
        self._odom_check_timer.cancel()

    # ==========================================
    # SISTEMA DE TELEMETRÍA (CRONÓMETRO Y VUELTAS)
    # ==========================================
    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = current_x
            self.start_y = current_y
            self.last_x = current_x
            self.last_y = current_y
            self.lap_start_time = time.time()
            return

        dx = current_x - self.last_x
        dy = current_y - self.last_y
        self.distance_traveled += math.sqrt(dx**2 + dy**2)
        self.last_x = current_x
        self.last_y = current_y

        if self.distance_traveled > 15.0:
            self.left_start_zone = True

        dist_to_start = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)
        if self.left_start_zone and dist_to_start < 2.0:
            lap_time = time.time() - self.lap_start_time
            self.lap_count += 1

            # NOTA: usamos self.get_logger().info() en vez de print().
            # print() puede quedar atrapado en el buffer de stdout cuando ROS 2
            # corre por debajo (sobre todo con 'ros2 launch' o salida a archivo
            # de log), y podrías terminar sin evidencia visible en el video.
            # El logger de ROS se imprime siempre, con timestamp, y es lo
            # estándar para mostrar evidencia en pantalla durante la grabación.
            self.get_logger().info(
                f"\n{'='*40}\n"
                f"🏁 [VUELTA {self.lap_count} COMPLETADA]\n"
                f"⏱️  Tiempo de vuelta: {lap_time:.2f} segundos\n"
                f"{'='*40}"
            )

            self.lap_start_time = time.time()
            self.distance_traveled = 0.0
            self.left_start_zone = False

    # ==========================================
    # CEREBRO REACTIVO: FOLLOW THE GAP (RÚBRICA PARTE A)
    # ==========================================
    def preprocess_lidar(self, ranges):
        """ Filtra valores infinitos y nulos del LiDAR. """
        proc_ranges = np.array(ranges)
        proc_ranges[np.isinf(proc_ranges)] = 10.0
        proc_ranges[np.isnan(proc_ranges)] = 0.0
        return proc_ranges

    def smooth_ranges(self, ranges):
        """
        Filtro de Promedio Móvil (Moving Average) sobre el LiDAR.
        CAUSA RAÍZ #1 del zangoloteo: el índice del punto "más profundo" salta
        de un rayo a otro entre escaneos consecutivos solo por ruido del sensor,
        aunque la pista no haya cambiado. Suavizar el escaneo ANTES de buscar
        gaps elimina ese salto en la fuente, en vez de parchearlo después.
        """
        w = self.lidar_filter_window
        kernel = np.ones(w) / w
        return np.convolve(ranges, kernel, mode='same')

    def scan_callback(self, msg):
        ranges = self.preprocess_lidar(msg.ranges)
        ranges = self.smooth_ranges(ranges)  # <-- NUEVO: denoising antes de cualquier cálculo

        # 1. VISIÓN DE TÚNEL (Evita dar giros en U)
        start_idx = int(len(ranges) * 0.17)
        end_idx = int(len(ranges) * 0.83)
        front_ranges = ranges[start_idx:end_idx]

        # PASO 1: Encontrar el obstáculo más cercano
        closest_idx = np.argmin(front_ranges)
        closest_dist = front_ranges[closest_idx]

        # PASO 2: Burbuja de Seguridad Dinámica
        angle_inc = msg.angle_increment
        
        # El auto se percibe más "gordo" mientras más lejos esté el obstáculo.
        effective_width = min(0.65, self.car_width + (0.05 * closest_dist))
        bubble_angle = math.atan2(effective_width / 2.0, max(closest_dist, 0.05))
        rays_to_eliminate = int(bubble_angle / angle_inc)

        # Esto "dibuja" el obstáculo en el cerebro del auto.
        bubble_start = max(0, closest_idx - rays_to_eliminate)
        bubble_end = min(len(front_ranges), closest_idx + rays_to_eliminate)
        front_ranges[bubble_start:bubble_end] = 0.0

        # PASO 3: Encontrar las brechas (Gaps)
        non_zero_indices = np.where(front_ranges > 0.0)[0]
        if len(non_zero_indices) == 0:
            # INSTINTO DE SUPERVIVENCIA
            self.get_logger().warn("⚠️ VISIÓN BLOQUEADA: Obstáculo encima. Avanzando a ciegas...")
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 1.0   
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        gaps = np.split(non_zero_indices, np.where(np.diff(non_zero_indices) != 1)[0] + 1)
        largest_gap = max(gaps, key=len)

        # PASO 4: TRAZADA AL CENTRO DEL HUECO (Pura supervivencia)
        # Apuntar al centro matemático del hueco garantiza la máxima distancia a las paredes.
        gap_ranges = front_ranges[largest_gap] 
        max_val = np.max(gap_ranges)

        # Encontramos dónde está la máxima profundidad (el horizonte)
        deep_indices = np.where(gap_ranges >= (max_val - 0.5))[0] 
        stable_max_depth_idx = int(np.mean(deep_indices))
        
        # Encontramos el centro geométrico (para alejarnos de los muros)
        center_of_gap = len(largest_gap) // 2

        # Fórmula de compromiso estático:
        weight_center = 0.60
        weight_depth = 0.40

        best_idx_in_gap = int((weight_depth * stable_max_depth_idx) + (weight_center * center_of_gap))
        best_idx = largest_gap[best_idx_in_gap]

        real_idx = start_idx + best_idx
        raw_steering_angle = msg.angle_min + (real_idx * msg.angle_increment)
        # ==========================================
        # CAUSA RAÍZ #3: CONTROL DE DIRECCIÓN ESTABILIZADO (PD + Slew-Rate Limiter)
        # Reemplaza el alpha switcheado (que se desactivaba justo en curvas cerradas).
        # ==========================================

        # Zona muerta: ignora micro-correcciones cuando casi va recto
        if abs(raw_steering_angle) < self.steering_deadband:
            raw_steering_angle = 0.0

        # --- Etapa A: Controlador PD ---
        # Setpoint = raw_steering_angle (lo que pide el Follow The Gap este frame)
        # Estado actual = lo último que de verdad mandamos al auto
        error = raw_steering_angle
        derivative = error - self.prev_error

        # Calculamos el giro candidato directo
        candidate_steering = (self.Kp * error) + (self.Kd * derivative)
        
        self.prev_error = error

        # --- Etapa B: Slew-Rate Limiter ADAPTATIVO ---
        t_rate = float(np.clip(
            (abs(raw_steering_angle) - self.angle_trigger_min) /
            (self.angle_trigger_max - self.angle_trigger_min), 0.0, 1.0
        ))
        dynamic_max_rate = self.rate_open + t_rate * (self.rate_tight - self.rate_open)

        # Limitamos qué tan rápido puede moverse físicamente el servo desde su posición anterior
        delta = candidate_steering - self.prev_steering_angle
        delta = float(np.clip(delta, -dynamic_max_rate, dynamic_max_rate))

        smoothed_steering = self.prev_steering_angle + delta
        self.prev_steering_angle = smoothed_steering

        # PASO 5: ACELERADOR Y RADAR PRE-CURVA
        # Extraemos la distancia promedio directamente frente al auto (aprox. 0 grados)
        center_ray_idx = len(front_ranges) // 2
        front_dist = np.mean(front_ranges[center_ray_idx-15 : center_ray_idx+15])

        abs_steer = abs(raw_steering_angle)

        if closest_dist < self.emergency_dist:
            # Obstáculo encima
            speed = 2.0
        else:
            # LÓGICA DE COMPETICIÓN:
            # Si la pared de enfrente está a menos de 4m O estamos dando un giro brusco -> Freno fuerte
            if front_dist < 7.0 or abs_steer > 0.25:
                speed = 3.5
            # Si la pared de enfrente está a menos de 7m O empezamos a girar el volante -> Soltar acelerador
            elif front_dist < 7.0 or abs_steer > 0.12:
                speed = 4.5
            # Recta despejada -> A fondo
            else:
                speed = 7.5

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = smoothed_steering
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl+C detectado. Aplicando freno de emergencia...")
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        for _ in range(5):
            node.drive_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()