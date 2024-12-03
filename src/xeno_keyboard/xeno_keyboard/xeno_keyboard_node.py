import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pynput import keyboard


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__("keyboard_control_node")
        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        self.joint_state = JointState()
        self.joint_state.name = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        self.joint_state.position = [0.0] * 6
        self.get_logger().info("Keyboard Control Node has been started.")

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == "q":
                self.joint_state.position[0] += 0.5
            elif key.char == "a":
                self.joint_state.position[0] -= 0.5
            elif key.char == "w":
                self.joint_state.position[1] += 0.5
            elif key.char == "s":
                self.joint_state.position[1] -= 0.5
            elif key.char == "e":
                self.joint_state.position[2] += 0.5
            elif key.char == "d":
                self.joint_state.position[2] -= 0.5
            elif key.char == "r":
                self.joint_state.position[3] += 0.5
            elif key.char == "f":
                self.joint_state.position[3] -= 0.5
            elif key.char == "t":
                self.joint_state.position[4] += 0.5
            elif key.char == "g":
                self.joint_state.position[4] -= 0.5
            elif key.char == "y":
                self.joint_state.position[5] += 0.5
            elif key.char == "h":
                self.joint_state.position[5] -= 0.5
            self.publisher_.publish(self.joint_state)
        except AttributeError:
            pass

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
