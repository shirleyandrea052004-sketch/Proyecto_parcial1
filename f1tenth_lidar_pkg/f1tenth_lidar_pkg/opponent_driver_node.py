#!/usr/bin/env python3
"""
opponent_driver_node.py
========================
Nodo MÍNIMO para mover al vehículo oponente como obstáculo dinámico
(punto 4 de tu rúbrica). No necesita ser inteligente: la rúbrica solo pide
que se mueva "a velocidad moderada" mientras tu Follow The Gap lo esquiva.

IMPORTANTE (documentado en el repo del simulador):
Si publicas en /drive pero NO simultáneamente en /opp_drive, el simulador
NO mueve a NINGUNO de los dos vehículos, ni siquiera al tuyo. Por eso este
nodo debe correr en una terminal aparte, AL MISMO TIEMPO que tu nodo
FollowTheGapNode, mientras el simulador está corriendo.

REQUISITO PREVIO:
En sim.yaml, cambia:
    num_agent: 2
para que aparezca el vehículo oponente en la simulación.

USO:
    # Terminal 1: simulador
    ros2 launch f1tenth_gym_ros gym_bridge_launch.py

    # Terminal 2: tu controlador
    ros2 run <tu_paquete> follow_the_gap_node

    # Terminal 3: el oponente (este script)
    ros2 run <tu_paquete> opponent_driver_node

NOTA SOBRE "2 ROBOTS ADICIONALES":
Este bridge (según su propia documentación) solo soporta 1 vehículo
adicional además del tuyo (num_agent: 1 o 2), es decir, máximo 2 vehículos
en total. Si tu rúbrica pide 2 robots adicionales (3 en total), eso podría
requerir una versión modificada del bridge o coordinarse con tu docente/
grupo sobre cómo se espera lograr esa parte — vale la pena confirmarlo
antes de invertir tiempo intentando forzarlo en este paquete.
"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class OpponentDriverNode(Node):
    def __init__(self):
        super().__init__('opponent_driver_node')
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)

        # --- Velocidad moderada y giro fijo, simple y predecible ---
        # Ajusta estos dos valores según el mapa: una velocidad moderada
        # (ni tan lenta que no sea un reto, ni tan rápida que sea imposible
        # de esquivar) y un giro ligero para que recorra la pista en vez de
        # quedarse fijo apuntando a una pared.
        self.moderate_speed = 1.0       # m/s
        self.steering_angle = 0.30       # rad (0.0 = recto; ajusta si tu mapa lo requiere)

        timer_period = 0.05  # 20 Hz, suficiente para un comando de velocidad constante
        self.timer = self.create_timer(timer_period, self.publish_drive)

        self.get_logger().info(
            f"Oponente iniciado: publicando en /opp_drive a {self.moderate_speed} m/s constante."
        )

    def publish_drive(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.moderate_speed
        msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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