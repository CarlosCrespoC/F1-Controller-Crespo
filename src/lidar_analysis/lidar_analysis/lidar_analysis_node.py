#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class LidarAnalysisNode(Node):
    def __init__(self):
        super().__init__('lidar_analysis_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10) # QoS de 10 es suficiente
        
        self.last_time = None
        self.results_printed = False

    def scan_callback(self, msg):
        if self.results_printed:
            return

        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return

        # --- PARTE CRÍTICA Y RÁPIDA ---
        # Mide el tiempo y calcula la frecuencia INMEDIATAMENTE.
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        frequency = 1.0 / (dt + 1e-9)
        
        # --- PARTE LENTA (PERO YA NO AFECTA LA MEDICIÓN) ---
        # Ahora que ya tenemos la frecuencia, hacemos el resto de cálculos.
        
        angular_resolution_deg = math.degrees(msg.angle_increment)
        
        front_ranges = []
        rear_ranges = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle) % 360
            
            if angle_deg >= 0 and angle_deg <= 180:
                front_ranges.append(distance)
            else:
                rear_ranges.append(distance)

        # --- MOSTRAR RESULTADOS ---
        self.get_logger().info("--- Análisis del LiDAR F1Tenth (Versión Optimizada) ---")
        self.get_logger().info(f"Resolución angular del LiDAR: {angular_resolution_deg:.3f} grados")
        self.get_logger().info(f"Frecuencia de publicación estimada: {frequency:.2f} Hz") # ¡Ahora mostrará ~250 Hz!
        self.get_logger().info(f"Cantidad de mediciones frontales (0° a 180°): {len(front_ranges)}")
        self.get_logger().info(f"Primeros 5 valores frontales: {[f'{r:.2f}' for r in front_ranges[:5]]}")
        self.get_logger().info(f"Cantidad de mediciones traseras (180° a 360°): {len(rear_ranges)}")
        self.get_logger().info(f"Primeros 5 valores traseros: {[f'{r:.2f}' for r in rear_ranges[:5]]}")
        self.get_logger().info("-------------------------------------------------------")
        
        self.results_printed = True
        self.get_logger().info("Análisis completado. Apagando el nodo...")
        time.sleep(1)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()