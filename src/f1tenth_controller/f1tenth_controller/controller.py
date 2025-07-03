#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Ejemplo de mensaje para publicar
from ackermann_msgs.msg import AckermannDriveStamped # Para comandos de control de F1Tenth
from sensor_msgs.msg import LaserScan # Para datos del lidar

class F1TenthController(Node):

    def __init__(self):
        super().__init__('f1tenth_controller_node')
        self.publisher_ = self.create_publisher(String, 'topic_a_publicar', 10) # Ejemplo de publicador
        self.subscription = self.create_subscription(
            LaserScan,
            'scan', # Tópico del lidar, común en F1Tenth
            self.laser_callback,
            10)
        self.subscription # prevent unused variable warning

        self.ackermann_publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10) # Publicador para comandos de AckermannDrive

        self.get_logger().info('F1Tenth Controller Node ha sido iniciado.')
        self.timer = self.create_timer(0.5, self.timer_callback) # Temporizador para publicar periódicamente

    def timer_callback(self):
        msg = String()
        msg.data = 'Hola desde el controlador F1Tenth! %d' % self.get_clock().now().nanoseconds
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando: "%s"' % msg.data)

        # Aquí es donde iría tu lógica de control para publicar a 'drive'
        # Por ejemplo, para mover hacia adelante:
        # drive_msg = AckermannDriveStamped()
        # drive_msg.drive.speed = 1.0 # Velocidad en m/s
        # drive_msg.drive.steering_angle = 0.0 # Ángulo de dirección en radianes
        # self.ackermann_publisher_.publish(drive_msg)

    def laser_callback(self, msg):
        # Aquí procesarías los datos del lidar (msg.ranges, msg.angle_min, etc.)
        self.get_logger().info('Recibido dato de LaserScan. Primer rango: %f' % msg.ranges[0])
        # Basado en estos datos, podrías calcular la velocidad y el ángulo de dirección
        # y luego publicar a self.ackermann_publisher_
        pass

def main(args=None):
    rclpy.init(args=args)
    f1tenth_controller = F1TenthController()
    rclpy.spin(f1tenth_controller)
    f1tenth_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
