#!/usr/bin/env python3
# Indica que este archivo se puede ejecutar como un script en sistemas tipo Unix.

# Importamos las bibliotecas necesarias para ROS 2, control, sensores y cálculos
import rclpy  # Biblioteca principal de ROS 2 en Python
from rclpy.node import Node  # Clase base para crear nodos en ROS
from sensor_msgs.msg import LaserScan  # Tipo de mensaje para datos de LiDAR
from nav_msgs.msg import Odometry  # Tipo de mensaje para odometría (posición y velocidad del robot)
from ackermann_msgs.msg import AckermannDriveStamped  # Tipo de mensaje para controlar vehículos Ackermann (como un auto)
import numpy as np  # Librería para cálculos numéricos (usamos principalmente para distancias)
import time  # Para controlar tiempos y cronómetros

# Definimos la clase principal que maneja el comportamiento del vehículo en carrera
class ControladorCarrera(Node):
    def __init__(self):
        super().__init__('controlador_carrera')  # Se registra el nodo con ese nombre en ROS 2

        # Suscribimos el nodo al sensor LiDAR y a la odometría del auto
        self.create_subscription(LaserScan, '/scan', self.procesar_lidar, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.procesar_odom, 10)

        # Creamos el publicador para enviar comandos de velocidad y giro
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Variables para el conteo de vueltas
        self.vueltas = -1  # Comenzamos en -1 para que la vuelta 0 sea la de inicio
        self.umbral_distancia = 2.0  # Distancia mínima para considerar que hemos "regresado al inicio"
        self.ya_paso_por_origen = False  # Bandera para evitar múltiples conteos seguidos

        # Temporizador para el cronómetro por vueltas
        self.inicio_vuelta = time.time()  # Marca el momento de inicio de la primera vuelta
        self.cronometro_timer = self.create_timer(1.0, self.mostrar_tiempo_parcial)  # Llama cada 1 segundo a mostrar el tiempo parcial

        # Parámetros del controlador PID para dirección del vehículo
        self.kp = 1.8  # Ganancia proporcional (controla respuesta rápida)
        self.kd = 0.8  # Ganancia derivativa (controla suavidad de correcciones)
        self.max_angulo = 0.4  # Límite máximo del ángulo de giro (en radianes)
        self.zona_muerta = 0.01  # Si el error es muy pequeño, no corregimos (evita vibraciones)
        self.error_anterior = 0.0  # Guarda el error anterior para cálculo derivativo
        self.tiempo_anterior = time.time()  # Marca de tiempo para derivada

        # Guarda la velocidad lineal del auto (para fines de monitoreo)
        self.velocidad_actual = 0.0

        # Mensaje de confirmación de inicio del nodo
        self.get_logger().info("Controlador iniciado")

    # -------------------------
    # PROCESAMIENTO DE LA ODOMETRÍA
    # -------------------------
    def procesar_odom(self, msg):
        # Obtenemos la posición actual del vehículo
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Calculamos la distancia desde el punto de origen (0,0)
        distancia = np.hypot(x, y)

        # Calculamos la velocidad lineal actual (por si queremos mostrarla)
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        self.velocidad_actual = np.hypot(vel_x, vel_y)

        # Si estamos cerca del punto inicial y no habíamos pasado antes
        if distancia < self.umbral_distancia and not self.ya_paso_por_origen:
            self.vueltas += 1  # Contamos la vuelta
            tiempo_actual = time.time()
            duracion = tiempo_actual - self.inicio_vuelta  # Calculamos duración

            # Imprimimos el tiempo de vuelta (a partir de la vuelta 1)
            if self.vueltas > 0:
                minutos = int(duracion // 60)
                segundos = int(duracion % 60)
                self.get_logger().info(f" Vuelta {self.vueltas} completada en {minutos}:{segundos:02d} minutos")
            else:
                self.get_logger().info(f" Vuelta {self.vueltas} (inicio)")

            # Reiniciamos el cronómetro y actualizamos la bandera
            self.inicio_vuelta = tiempo_actual
            self.ya_paso_por_origen = True

        # Si estamos lejos del punto de origen, permitimos que se cuente otra vuelta después
        elif distancia >= self.umbral_distancia:
            self.ya_paso_por_origen = False

    # -------------------------
    # MOSTRAR TIEMPO PARCIAL
    # -------------------------
    def mostrar_tiempo_parcial(self):
        # Se ejecuta cada segundo. Muestra cuánto ha durado la vuelta actual.
        if self.vueltas >= 0:
            tiempo_actual = time.time() - self.inicio_vuelta
            minutos = int(tiempo_actual // 60)
            segundos = int(tiempo_actual % 60)
            self.get_logger().info(f" Tiempo parcial {self.vueltas}: {minutos}:{segundos:02d}")
            self.get_logger().info(f" Velocidad actual: {self.velocidad_actual:.2f} m/s")

    # -------------------------
    # PROCESAMIENTO DEL LIDAR
    # -------------------------
    def procesar_lidar(self, msg):
        # Limpiamos y normalizamos los datos del LiDAR
        rangos = np.nan_to_num(msg.ranges, nan=msg.range_max)  # Reemplaza NaNs por el máximo valor
        rangos = np.clip(rangos, msg.range_min, msg.range_max)  # Asegura que estén en el rango válido

        # Definimos algunos valores clave
        N = len(rangos)
        centro = N // 2  # Dirección completamente al frente
        incremento = msg.angle_increment  # Resolución angular del LiDAR
        ancho = 135  # Cantidad de lecturas a tomar a izquierda y derecha del frente
        frente = rangos[centro - ancho:centro + ancho]  # Ventana de atención frontal

        # --- DETECCIÓN DE EMERGENCIA (si hay algo demasiado cerca al frente) ---
        ventana_emergencia = 10  # Número de lecturas centrales para evaluar
        centro_frente = len(frente) // 2
        min_frente = np.min(frente[centro_frente - ventana_emergencia : centro_frente + ventana_emergencia])

        if min_frente < 0.5:
            # Obstáculo muy cerca → frenamos completamente
            return self.enviar_comando(0.0, 0.0)
        elif min_frente < 0.8:
            # Obstáculo cerca pero no crítico → evasión lateral
            izquierda = np.mean(rangos[centro + 30:centro + 90])
            derecha = np.mean(rangos[centro - 90:centro - 30])
            angulo = 0.4 if izquierda > derecha else -0.4
            return self.enviar_comando(1.0, angulo)

        # --- DETECCIÓN DE GAPS (huecos donde el auto puede pasar) ---
        gaps = []
        i = 0
        while i < len(frente):
            if frente[i] > 1.5:
                ini = i
                while i < len(frente) and frente[i] > 1.5:
                    i += 1
                gaps.append((ini, i - 1))  # Guardamos el inicio y fin del hueco
            else:
                i += 1

        if not gaps:
            # Si no hay ningún hueco seguro → nos detenemos
            return self.enviar_comando(0.0, 0.0)

        # Elegimos el mejor hueco (el más grande y cercano al centro visual)
        mejor_gap = max(gaps, key=lambda g: (g[1] - g[0]) - 0.5 * abs(((g[0] + g[1]) // 2) - ancho))
        centro_gap = (mejor_gap[0] + mejor_gap[1]) // 2
        idx_lidar = centro_gap + (centro - ancho)

        # --- CONTROLADOR PID PARA GIRAR HACIA EL GAP ---
        error = (idx_lidar - centro) * incremento  # Error en radianes respecto al frente
        if abs(error) < self.zona_muerta:
            error = 0.0  # Evita ajustes si el error es muy pequeño

        t_actual = time.time()
        dt = t_actual - self.tiempo_anterior if self.tiempo_anterior else 1e-3
        derivada = (error - self.error_anterior) / dt if dt > 0 else 0.0

        angulo = self.kp * error + self.kd * derivada  # Cálculo del ángulo con PID
        angulo = np.clip(angulo, -self.max_angulo, self.max_angulo)  # Lo limitamos

        self.error_anterior = error
        self.tiempo_anterior = t_actual

        # --- AJUSTE DE VELOCIDAD SEGÚN CURVATURA Y OBSTÁCULOS ---
        abs_ang = abs(angulo)

        # En rectas con mucho espacio, aceleramos más
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
            velocidad = 2.0  # Curvas cerradas → muy lento

        self.enviar_comando(velocidad, angulo)

    # -------------------------
    # ENVÍO DEL COMANDO FINAL DE MOVIMIENTO
    # -------------------------
    def enviar_comando(self, velocidad, angulo):
        # Prepara y publica el mensaje de velocidad y ángulo al vehículo
        msg = AckermannDriveStamped()
        msg.drive.speed = float(velocidad)
        msg.drive.steering_angle = float(angulo)
        self.pub.publish(msg)

# -------------------------
# FUNCIÓN PRINCIPAL DE ROS 2
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ControladorCarrera()
    try:
        rclpy.spin(node)  # Ejecutamos el nodo
    finally:
        node.destroy_node()
        rclpy.shutdown()  # Apagamos ROS

# Si este archivo se ejecuta directamente, llamamos a main()
if __name__ == '__main__':
    main()
