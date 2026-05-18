import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import socket
import json
import threading

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_ros_node')

        self.pub_vitesse = self.create_publisher(Float32, '/vitesse_reelle', 10)
        self.pub_acceleration = self.create_publisher(Float32, '/acceleration_reelle', 10)
        self.pub_odom_reelle = self.create_publisher(Odometry, '/odom_reelle', 10)
        self.pub_marker = self.create_publisher(Marker, '/vehicle_reelle', 10)

        self.position = 0.0

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(('0.0.0.0', 5000))
        self.server.listen(1)
        self.get_logger().info('En attente du Raspberry Pi sur port 5000...')

        thread = threading.Thread(target=self.listen)
        thread.daemon = True
        thread.start()

    def listen(self):
        conn, addr = self.server.accept()
        self.get_logger().info(f'Raspberry Pi connecté : {addr}')

        buffer = ""
        while True:
            try:
                data = conn.recv(1024).decode()
                if not data:
                    break
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    parsed = json.loads(line)

                    msg_v = Float32(); msg_v.data = parsed["vitesse"]
                    self.pub_vitesse.publish(msg_v)

                    msg_a = Float32(); msg_a.data = parsed["acceleration"]
                    self.pub_acceleration.publish(msg_a)

                    self.position += parsed["vitesse"] * 1.0

                    # Odometry
                    odom = Odometry()
                    odom.header.stamp = self.get_clock().now().to_msg()
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'base_link'
                    odom.pose.pose.position.x = self.position
                    odom.pose.pose.position.y = 0.3
                    odom.pose.pose.position.z = 0.0
                    odom.twist.twist.linear.x = parsed["vitesse"]
                    self.pub_odom_reelle.publish(odom)

                    # Marker robot réel (bleu)
                    marker = Marker()
                    marker.header.frame_id = 'odom'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'vehicle_real'
                    marker.id = 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = self.position
                    marker.pose.position.y = 0.3
                    marker.pose.position.z = 0.05
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.2
                    marker.scale.y = 0.1
                    marker.scale.z = 0.05
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0
                    self.pub_marker.publish(marker)

                    self.get_logger().info(
                        f'Reçu → Vitesse: {parsed["vitesse"]} m/s | Accel: {parsed["acceleration"]} m/s²'
                    )
            except Exception as e:
                self.get_logger().error(f'Erreur: {e}')
                break

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
