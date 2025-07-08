#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry # Asegúrate de que esta importación esté presente
import numpy as np
import math

class EmergencyBrakeNode(Node):
    """
    Nodo ROS 2 con freno de emergencia y velocidad forzada a 3 m/s.
    ¡Mejorada la detección de obstáculos para evitar frenados innecesarios por paredes laterales!
    """
    def __init__(self):
        super().__init__('emergency_brake_node')

        # --- Parámetros de Seguridad ---
        # Estos parámetros nos permiten ajustar el comportamiento del freno de emergencia.
        # Los hacemos 'declarables' para poder cambiarlos sin modificar el código directamente,
        # lo que es genial para probar y afinar.
        self.declare_parameter('slowdown_distance', 1.5) # Distancia para empezar a reducir la velocidad (en metros)
        self.declare_parameter('stop_distance', 0.5)    # Distancia para detenerse por completo (en metros)
        self.declare_parameter('slowdown_speed', 0.8)   # Velocidad a la que el robot desacelera (en m/s)
        
        # Este es crucial para la detección de paredes. Es el ángulo a CADA LADO del frente del robot
        # que consideraremos para la detección de obstáculos. Un valor más pequeño (ej. 20-30 grados)
        # es mejor para ignorar paredes laterales.
        self.declare_parameter('lidar_angle_range_deg', 30.0) # 30 grados para un cono más estrecho
        
        # Nuevo parámetro: Número mínimo de puntos LiDAR consecutivos para considerar un obstáculo.
        # Esto ayuda a filtrar el ruido o lecturas aisladas de paredes que no son un problema.
        self.declare_parameter('min_consecutive_points_for_obstacle', 3) 

        # Ahora obtenemos los valores de esos parámetros.
        self.SLOWDOWN_DISTANCE = self.get_parameter('slowdown_distance').get_parameter_value().double_value
        self.STOP_DISTANCE = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.SLOWDOWN_SPEED = self.get_parameter('slowdown_speed').get_parameter_value().double_value
        self.LIDAR_ANGLE_RANGE_DEG = self.get_parameter('lidar_angle_range_deg').get_parameter_value().double_value
        self.MIN_CONSECUTIVE_POINTS_FOR_OBSTACLE = self.get_parameter('min_consecutive_points_for_obstacle').get_parameter_value().integer_value
        
        # Esta variable guardará el último comando de velocidad que nos dio el teleoperador (teclado).
        # Lo necesitamos para saber qué querías hacer antes de que el freno de emergencia interviniera.
        self.last_teleop_cmd = None

        # --- Suscripciones y Publicadores ---
        # ¡Aquí es donde nuestro nodo 'escucha' y 'habla' con el resto del sistema ROS!

        # Suscriptor para los comandos de velocidad del teleoperador (desde el teclado).
        # Es importante que el nodo del teclado publique en '/cmd_vel_raw' para que lo interceptemos.
        self.teleop_sub = self.create_subscription(
            Twist,              # Tipo de mensaje: Twist (velocidad lineal y angular)
            '/cmd_vel_raw',     # Tópico al que nos suscribimos
            self.teleop_callback,# Función que se llamará cuando recibamos un mensaje
            10)                 # QoS: tamaño del buffer de mensajes (cantidad de mensajes a guardar)

        # Suscriptor para los datos del sensor LiDAR.
        # El LiDAR nos da las distancias a los obstáculos.
        self.scan_sub = self.create_subscription(
            LaserScan,          # Tipo de mensaje: LaserScan
            '/scan',            # Tópico del LiDAR
            self.scan_callback, # Función que procesa los datos del LiDAR
            10)

        # Suscriptor para los datos de odometría del robot.
        # Esto nos da la posición estimada del robot.
        self.odom_sub = self.create_subscription(
            Odometry,           # Tipo de mensaje: Odometry
            '/odom',            # Tópico de odometría
            self.odom_callback, # Función que mostrará la odometría completa
            10)

        # Publicador para los comandos de velocidad "seguros" y finales.
        # Estos comandos irán directamente al control de motores del robot.
        self.safe_cmd_pub = self.create_publisher(
            Twist,              # Tipo de mensaje: Twist
            '/cmd_vel',         # Tópico donde publicamos los comandos seguros
            10)
        
        # Un mensaje para saber que nuestro nodo se ha iniciado correctamente.
        self.get_logger().info('Nodo de Freno de Emergencia iniciado. ¡Listo para una conducción más segura!')

    # --- Funciones Callback ---
    # Estas funciones se activan automáticamente cuando se recibe un mensaje en sus respectivos tópicos.

    def teleop_callback(self, msg):
        """
        Esta función se activa cada vez que recibimos un comando de velocidad del teleoperador (teclado).
        Aquí, guardamos el comando y nos aseguramos de que la velocidad de avance sea siempre 3 m/s.
        """
        # Creamos un nuevo mensaje Twist para evitar modificar el original directamente.
        new_cmd = Twist()
        new_cmd.linear.x = msg.linear.x    # Copiamos la velocidad lineal
        new_cmd.angular.z = msg.angular.z  # Copiamos la velocidad angular

        # Si el teleoperador nos pide avanzar (velocidad lineal positiva)...
        if new_cmd.linear.x > 0.0:
            # ...forzamos la velocidad lineal a 3.0 m/s.
            # Esto significa que no importa cuán "fuerte" presiones la tecla de avance,
            # el robot siempre intentará ir a 3 m/s, a menos que el freno de emergencia intervenga.
            new_cmd.linear.x = 3.0

        # Guardamos este comando (ya modificado si era de avance) para usarlo en la lógica del freno.
        self.last_teleop_cmd = new_cmd

    def odom_callback(self, msg: Odometry): # Se agregó el tipo de dato para mayor claridad
        """
        Esta función se activa con cada actualización de la odometría del robot.
        Ahora extrae y muestra la posición actual Y LA VELOCIDAD LINEAL en el terminal.
        """
        # Obtenemos las coordenadas X e Y de la posición del robot.
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        # Obtenemos la velocidad lineal del robot en el eje X (avance/retroceso).
        vel_x = msg.twist.twist.linear.x
        
        # Mostramos la posición y la velocidad. 
        # 'throttle_duration_sec' evita que inunde el terminal mostrando demasiados mensajes.
        self.get_logger().info(
            f'Odometría - Posición: (X: {pos_x:.2f} m, Y: {pos_y:.2f} m) | Velocidad Lineal: {vel_x:.2f} m/s',
            throttle_duration_sec=0.5 # Mensajes cada 0.5 segundos para un buen flujo
        )

    def scan_callback(self, msg):
        """
        Esta es la función principal del freno de emergencia. Se activa con cada lectura del LiDAR.
        Analiza los datos del LiDAR para detectar obstáculos y ajusta la velocidad del robot si es necesario.
        """
        # Si aún no hemos recibido un comando del teleoperador, no hay nada que filtrar, así que salimos.
        if self.last_teleop_cmd is None:
            return

        # Creamos una copia del último comando del teleoperador. Esta es la velocidad "deseada"
        # antes de aplicar las reglas de seguridad.
        safe_cmd = self.last_teleop_cmd

        # --- Procesamiento de los datos del LiDAR para detección frontal ---

        # Calculamos el rango angular del LiDAR que nos interesa (en radianes).
        # Este es nuestro "cono de visión" frontal para los obstáculos.
        angle_range_rad = math.radians(self.LIDAR_ANGLE_RANGE_DEG)
        
        # El índice central en el array de lecturas del LiDAR (generalmente el frente del robot).
        center_index = len(msg.ranges) // 2
        
        # Calculamos cuántos índices a cada lado del centro debemos buscar.
        search_width_indices = int((angle_range_rad / 2.0) / msg.angle_increment)

        # Definimos los índices de inicio y fin para el segmento frontal del LiDAR.
        # Usamos max(..., 0) y min(..., len-1) para asegurarnos de no salirnos de los límites del array.
        start_index = max(center_index - search_width_indices, 0)
        end_index = min(center_index + search_width_indices, len(msg.ranges) - 1)

        # Convertimos las lecturas del LiDAR a un array de NumPy para facilitar el procesamiento.
        ranges = np.array(msg.ranges)
        
        # Seleccionamos solo las lecturas dentro de nuestro cono frontal definido.
        front_ranges_raw = ranges[start_index : end_index + 1] 

        # Limpiamos las lecturas:
        # Los valores '0.0' o 'inf' suelen significar que no hay detección, fuera de rango, o error.
        # Los reemplazamos con un valor infinito para que no afecten el cálculo de la distancia mínima.
        front_ranges_filtered = front_ranges_raw[np.isfinite(front_ranges_raw)]
        
        # Si no hay puntos válidos en el rango frontal después de filtrar, no hay obstáculo detectado.
        if len(front_ranges_filtered) == 0:
            min_dist = np.inf
        else:
            # Ahora, la mejora clave: verificamos si hay suficientes puntos cercanos para considerar un obstáculo.
            # Esto ayuda a ignorar el ruido o las lecturas aisladas de una pared lateral.
            close_points = front_ranges_filtered[front_ranges_filtered < self.SLOWDOWN_DISTANCE]
            
            if len(close_points) >= self.MIN_CONSECUTIVE_POINTS_FOR_OBSTACLE:
                # Si hay suficientes puntos cercanos, encontramos la distancia mínima entre ellos.
                min_dist = np.min(close_points)
            else:
                # Si no hay suficientes puntos válidos que cumplan la condición de cercanía,
                # asumimos que no hay un obstáculo significativo.
                min_dist = np.inf 

        # --- Lógica del Freno de Emergencia ---

        # La velocidad objetivo inicial es la que nos dio el teleoperador.
        target_speed = safe_cmd.linear.x

        # Primera condición: ¿Hay un obstáculo para detención total?
        if min_dist < self.STOP_DISTANCE:
            # Si el robot está intentando avanzar...
            if safe_cmd.linear.x > 0.0:
                self.get_logger().warn(f'¡PELIGRO! Obstáculo frontal a {min_dist:.2f} m. ¡DETENIENDO AVANCE!')
                target_speed = 0.0 # ¡Frena completamente!
            else:
                # Si el robot ya estaba retrocediendo, le permitimos seguir.
                self.get_logger().info(f'Obstáculo frontal muy cerca a {min_dist:.2f} m. Permitiendo retroceso.')
                target_speed = safe_cmd.linear.x
        
        # Segunda condición: ¿Hay un obstáculo para desaceleración?
        elif min_dist < self.SLOWDOWN_DISTANCE:
            # Si el robot está intentando avanzar...
            if safe_cmd.linear.x > 0.0:
                self.get_logger().info(f'¡Cuidado! Obstáculo frontal a {min_dist:.2f} m. Reduciendo velocidad.')
                # La nueva velocidad será la más baja entre la velocidad de desaceleración y la que pediste.
                # Esto evita que el robot acelere si ya iba muy lento.
                target_speed = min(self.SLOWDOWN_SPEED, safe_cmd.linear.x)
            else:
                # Si el robot ya estaba retrocediendo, le permitimos seguir.
                target_speed = safe_cmd.linear.x
        
        # Si no hay obstáculos cercanos, la velocidad objetivo es la que viene del teleoperador.
        # (Esto ya está implícito porque 'target_speed' se inicializa con 'safe_cmd.linear.x' y no se modifica
        # si no se cumplen las condiciones de freno).

        # Asignamos la velocidad lineal segura (calculada o la original) al comando final.
        safe_cmd.linear.x = target_speed

        # Publicamos el comando de velocidad final y seguro al robot.
        self.safe_cmd_pub.publish(safe_cmd)

# --- Función Principal (Entry Point) ---
def main(args=None):
    # Inicializamos ROS 2. Esto es lo primero que se debe hacer.
    rclpy.init(args=args)
    
    # Creamos una instancia de nuestro nodo EmergencyBrakeNode.
    node = EmergencyBrakeNode()
    
    # Mantenemos el nodo en funcionamiento, esperando mensajes y ejecutando las callbacks.
    # El nodo seguirá activo hasta que se detenga manualmente (ej. Ctrl+C).
    rclpy.spin(node)
    
    # Cuando el nodo se detiene, limpiamos los recursos.
    node.destroy_node()
    
    # Apagamos ROS 2.
    rclpy.shutdown()

# Esto asegura que la función 'main()' se ejecute cuando el script es llamado directamente.
if __name__ == '__main__':
    main()