#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time

class ControladorCarrera(Node):
    def __init__(self):
        super().__init__('controlador_carrera')

        self.create_subscription(LaserScan, '/scan', self.procesar_lidar, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.procesar_odom, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.vueltas = -1
        self.umbral_distancia = 2.0
        self.ya_paso_por_origen = False

        self.inicio_vuelta = time.time()
        self.cronometro_timer = self.create_timer(1.0, self.mostrar_tiempo_parcial)

        self.kp = 1.8
        self.kd = 0.8
        self.max_angulo = 0.4
        self.zona_muerta = 0.01
        self.error_anterior = 0.0
        self.tiempo_anterior = time.time()

        self.velocidad_actual = 0.0

        self.get_logger().info("Controlador iniciado")

    def procesar_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        distancia = np.hypot(x, y)

        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        self.velocidad_actual = np.hypot(vel_x, vel_y)

        if distancia < self.umbral_distancia and not self.ya_paso_por_origen:
            self.vueltas += 1
            tiempo_actual = time.time()
            duracion = tiempo_actual - self.inicio_vuelta

            if self.vueltas > 0:
                minutos = int(duracion // 60)
                segundos = int(duracion % 60)
                self.get_logger().info(f" Vuelta {self.vueltas} completada en {minutos}:{segundos:02d} minutos")
            else:
                self.get_logger().info(f" Vuelta {self.vueltas} (inicio)")

            self.inicio_vuelta = tiempo_actual
            self.ya_paso_por_origen = True

        elif distancia >= self.umbral_distancia:
            self.ya_paso_por_origen = False

    def mostrar_tiempo_parcial(self):
        if self.vueltas >= 0:
            tiempo_actual = time.time() - self.inicio_vuelta
            minutos = int(tiempo_actual // 60)
            segundos = int(tiempo_actual % 60)
            self.get_logger().info(f" Tiempo parcial {self.vueltas}: {minutos}:{segundos:02d}")
            self.get_logger().info(f" Velocidad actual: {self.velocidad_actual:.2f} m/s")

    def procesar_lidar(self, msg):
        rangos = np.nan_to_num(msg.ranges, nan=msg.range_max)
        rangos = np.clip(rangos, msg.range_min, msg.range_max)
        N, centro, incremento = len(rangos), len(rangos) // 2, msg.angle_increment
        ancho = 135
        frente = rangos[centro - ancho:centro + ancho]

        # Detección de emergencia
        ventana_emergencia = 10
        centro_frente = len(frente) // 2
        min_frente = np.min(frente[centro_frente - ventana_emergencia : centro_frente + ventana_emergencia])
        if min_frente < 0.5:
            return self.enviar_comando(0.0, 0.0)  # Detención crítica
        elif min_frente < 0.8:
            izquierda = np.mean(rangos[centro + 30:centro + 90])
            derecha = np.mean(rangos[centro - 90:centro - 30])
            angulo = 0.4 if izquierda > derecha else -0.4
            return self.enviar_comando(1.0, angulo)  # Evasión

        # Detección de gaps
        gaps = []
        i = 0
        while i < len(frente):
            if frente[i] > 1.5:
                ini = i
                while i < len(frente) and frente[i] > 1.5:
                    i += 1
                gaps.append((ini, i - 1))
            else:
                i += 1

        if not gaps:
            return self.enviar_comando(0.0, 0.0)

        mejor_gap = max(gaps, key=lambda g: (g[1] - g[0]) - 0.5 * abs(((g[0] + g[1]) // 2) - ancho))
        centro_gap = (mejor_gap[0] + mejor_gap[1]) // 2
        idx_lidar = centro_gap + (centro - ancho)

        # PID
        error = (idx_lidar - centro) * incremento
        if abs(error) < self.zona_muerta:
            error = 0.0

        t_actual = time.time()
        dt = t_actual - self.tiempo_anterior if self.tiempo_anterior else 1e-3
        derivada = (error - self.error_anterior) / dt if dt > 0 else 0.0
        angulo = self.kp * error + self.kd * derivada
        angulo = np.clip(angulo, -self.max_angulo, self.max_angulo)
        self.error_anterior = error
        self.tiempo_anterior = t_actual

        # Velocidad adaptativa con escalado fino para rectas
        abs_ang = abs(angulo)

        if abs_ang < 0.02:
            if min_frente > 15.0:
                velocidad = 14.0
            elif min_frente > 14.0:
                velocidad = 13.0
            elif min_frente > 12.0:
                velocidad = 12.0
            elif min_frente > 11.0:
                velocidad = 11.0
            elif min_frente > 9.0:
                velocidad = 10.0
            elif min_frente > 6.0:
                velocidad = 9.0
            else:
                velocidad = 5.0
        elif abs_ang < 0.04:
            velocidad = 8.0
        elif abs_ang < 0.06:
            velocidad = 7.0
        elif abs_ang < 0.08:
            velocidad = 6.0
        elif abs_ang < 0.10:
            velocidad = 5.0
        elif abs_ang < 0.15:
            velocidad = 4.0
        elif abs_ang < 0.2:
            velocidad = 3.0
        else:
            velocidad = 2.0

        self.enviar_comando(velocidad, angulo)

    def enviar_comando(self, velocidad, angulo):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(velocidad)
        msg.drive.steering_angle = float(angulo)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControladorCarrera()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
