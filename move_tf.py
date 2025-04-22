import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import math
from geometry_msgs.msg import Quaternion

class EarthTfBroadcaster(Node):
    def __init__(self):
        super().__init__('earth_tf_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)  # 10 Hz
        self.t = 0.0  # Tiempo para la curva

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def broadcast_transform(self):
        self.t += 0.01  # Pequeño incremento para que la curva sea lenta
        
        # Movimiento en espiral (curva en 3D)
        x = 2.0 * math.cos(self.t)  # Oscila en X
        y = 2.0 * math.sin(self.t)  # Oscila en Y
        z = 1.5  # Sube lentamente
        
        # Calcular ángulo de orientación en el plano XY
        yaw = math.atan2(2.0 * math.sin(self.t + 0.01) - y, 2.0 * math.cos(self.t + 0.01) - x)
        
        # Convertir a cuaternión
        q = self.euler_to_quaternion(0, 0, yaw)
        
        # Crear mensaje de transformación
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "earth"  # Frame de referencia global
        transform.child_frame_id = "world"  # Frame que queremos mover
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        
        # Asignar rotación calculada
        transform.transform.rotation = q
        
        # Publicar la transformación
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f'Publicado TF earth -> Pos ({x:.2f}, {y:.2f}, {z:.2f}), Yaw: {yaw:.2f}')


def main():
    rclpy.init()
    node = EarthTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
