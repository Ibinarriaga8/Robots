import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

class TeleoperationNode(Node):

    def __init__(self)->Node:
        super().__init__("teleoperation_node")
        self.vel_actual_x = 0.0
        self.vel_actual_z = 0.0
        self._publisher = self.create_publisher(msg_type=Twist, topic="/cmd_vel", qos_profile=10)
        self._subscriber = self.create_subscription(msg_type=String, topic="/keyboard_node/keyboard_pressed", callback=self._listener_callback, qos_profile=10)
        
        lidar_qos_profile = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10,
            durability = QoSDurabilityPolicy.VOLATILE,
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._lidar_subscriber = self.create_subscription(msg_type=LaserScan, topic="/scan", callback=self._lidar_callback, qos_profile=lidar_qos_profile)
        self.obstacle_distance = float('inf')  

    def _listener_callback(self, msg:String)->None:
        key = msg.data
        print(f"Tecla presionada: {key}, Distancia al obstáculo: {self.obstacle_distance}")
        if key == "w" :
            #self._lidar_callback(self.obstacle_distance)
            self.vel_actual_x += 0.1
        elif key == "s":
            #self._lidar_callback(self.obstacle_distance)
            self.vel_actual_x -= 0.1
        elif key == "a":
            #self._lidar_callback(self.obstacle_distance)
            self.vel_actual_z += 0.1
        elif key == "d":
            #self._lidar_callback(self.obstacle_distance)
            self.vel_actual_z -= 0.1
        elif key == "space":
            self.vel_actual_x = 0.0
            self.vel_actual_z = 0.0


        print(f"Velocidades actualizadas: Linear X: {self.vel_actual_x}, Angular Z: {self.vel_actual_z}")
        self.publish_velocity()

    def _lidar_callback(self, msg:LaserScan)->None:
        self.obstacle_distance = min(msg.ranges)
        # self.get_logger().info(f'Publishing: {msg.ranges}')
        if self.obstacle_distance < 0.2: 
            self.vel_actual_x = 0.0
            self.vel_actual_z = 0.0
            self.publish_velocity()

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.vel_actual_x
        twist.angular.z = self.vel_actual_z
        self._publisher.publish(twist)

def main(args=None)->None:
    rclpy.init(args=args)
    teleoperation_node = TeleoperationNode()
    rclpy.spin(teleoperation_node)
    teleoperation_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
