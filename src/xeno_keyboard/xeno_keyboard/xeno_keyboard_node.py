import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pynput import keyboard
from launch.substitutions import (
    PathJoinSubstitution,
)
from ament_index_python import get_package_share_directory
import xml.etree.ElementTree as ET


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
        self.joint_limits = self.get_joint_limits()
        self.get_logger().info("Keyboard Control Node has been started.")

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def get_joint_limits(self):
        joint_limits = {}
        urdf_path = os.path.join(
            get_package_share_directory("xeno_urdf"),
            "xeno_urdf",
            "urdf",
            "xeno_urdf_description.urdf.xacro",
        )
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        for joint in root.findall(".//joint"):
            name = joint.get("name")
            limit = joint.find("limit")
            if limit is not None:
                lower = float(limit.get("lower", "-inf"))
                upper = float(limit.get("upper", "inf"))
                joint_limits[name] = (lower, upper)
        return joint_limits

    def check_joint_limits(self, joint_name, position):
        lower, upper = self.joint_limits.get(joint_name, (-float("inf"), float("inf")))
        return lower <= position <= upper

    def on_press(self, key):
        try:
            if key.char == "[":
                self.joint_state.position[0] += 0.005
            elif key.char == "]":
                self.joint_state.position[0] -= 0.005
            elif key.char == "{":
                self.joint_state.position[0] += 0.1
            elif key.char == "}":
                self.joint_state.position[0] -= 0.1
            elif key.char == "8":
                self.joint_state.position[1] += 0.1
            elif key.char == "2":
                self.joint_state.position[1] -= 0.1
            elif key.char == "4":
                self.joint_state.position[2] += 0.1
            elif key.char == "6":
                self.joint_state.position[2] -= 0.1

            elif key.char == "z":
                self.joint_state.position[3] += 0.005
            elif key.char == "c":
                self.joint_state.position[3] -= 0.005
            elif key.char == "Z":
                self.joint_state.position[3] += 0.1
            elif key.char == "C":
                self.joint_state.position[3] -= 0.1

            elif key.char == "a":
                self.joint_state.position[4] -= 0.005
            elif key.char == "d":
                self.joint_state.position[4] += 0.005
            elif key.char == "A":
                self.joint_state.position[4] -= 0.1
            elif key.char == "D":
                self.joint_state.position[4] += 0.1

            elif key.char == "q":
                self.joint_state.position[5] -= 0.005
            elif key.char == "e":
                self.joint_state.position[5] += 0.005
            elif key.char == "Q":
                self.joint_state.position[5] -= 0.1
            elif key.char == "E":
                self.joint_state.position[5] += 0.1
        except AttributeError:
            if key == keyboard.Key.up:
                self.joint_state.position[1] += 0.005
            elif key == keyboard.Key.down:
                self.joint_state.position[1] -= 0.005
            elif key == keyboard.Key.left:
                self.joint_state.position[2] += 0.005
            elif key == keyboard.Key.right:
                self.joint_state.position[2] -= 0.005

        for i in range(6):
            joint_name = self.joint_state.name[i]
            if not self.check_joint_limits(joint_name, self.joint_state.position[i]):
                self.joint_state.position[i] = max(
                    min(self.joint_state.position[i], self.joint_limits[joint_name][1]),
                    self.joint_limits[joint_name][0],
                )

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

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
