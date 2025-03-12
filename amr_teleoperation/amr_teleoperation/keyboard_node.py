import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sshkeyboard import listen_keyboard

def press(key):
    print(f"Key {key} pressed")

def release(key):
    print(f"Key {key} released")

class KeyboardPublisher(Node):
    
    def __init__(self):
        super().__init__("keyboard_publisher")
        self.publisher = self.create_publisher(String, "/keyboard_node/keyboard_pressed", 10)
        self.start_keyboard_listener()

    def start_keyboard_listener(self):
        listen_keyboard(on_press=self.handle_key_press, on_release=release)

    def handle_key_press(self, key):
        msg = String()
        msg.data = key
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
