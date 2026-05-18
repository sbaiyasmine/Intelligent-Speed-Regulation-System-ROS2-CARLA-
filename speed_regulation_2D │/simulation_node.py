import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import numpy as np

class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_2d_node')

        self.pub_vitesse = self.create_publisher(Float32, '/vitesse_2d', 10)
        self.pub_acceleration = self.create_publisher(Float32, '/acceleration_2d', 10)
        self.pub_position = self.create_publisher(Float32, '/position_2d', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom_2d', 10)
        self.pub_marker = self.create_publisher(Marker, '/vehicle_2d', 10)

        self.dt = 0.05
        self.a_max = 0.047
        self.vitesse_cible = 0.1
        self.Kp = 2.0
        self.Ki = 0.5
        self.Kd = 0.1

        self.vitesse = 0.0
        self.position = 0.0
        self.erreur_precedente = 0.0
        self.integrale = 0.0

        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info('Simulation 2D démarrée !')

    def update(self):
        erreur = self.vitesse_cible - self.vitesse
        self.integrale += erreur * self.dt
        derivee = (erreur - self.erreur_precedente) / self.dt
        commande = self.Kp * erreur + self.Ki * self.integrale + self.Kd * derivee
        acceleration = float(np.clip(commande, -self.a_max, self.a_max))

        self.vitesse += acceleration * self.dt
        self.vitesse = max(0.0, self.vitesse)
        self.position += self.vitesse * self.dt
        self.erreur_precedente = erreur

        # Reset position après 5m
        if self.position > 5.0:
            self.position = 0.0

        # Float32
        msg_v = Float32(); msg_v.data = self.vitesse
        msg_a = Float32(); msg_a.data = acceleration
        msg_p = Float32(); msg_p.data = self.position
        self.pub_vitesse.publish(msg_v)
        self.pub_acceleration.publish(msg_a)
        self.pub_position.publish(msg_p)

        # Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.position
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x = self.vitesse
        self.pub_odom.publish(odom)

        # Marker cube rouge GRAND
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'vehicle_sim'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.position
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.3
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
