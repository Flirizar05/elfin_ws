#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateAligner(Node):
    def __init__(self):
        super().__init__('joint_state_aligner')
        # offsets con el robot en HOME físico (rad)
        self.offsets = {
            'elfin_joint1': 2.363,
            'elfin_joint2': 2.013,
            'elfin_joint3': 1.535,
            'elfin_joint4': 0.996,
            'elfin_joint5': 2.343,
            'elfin_joint6': 1.075,
        }
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.cb, 10
        )
        self.pub = self.create_publisher(
            JointState, '/joint_states_aligned', 10
        )
        self.get_logger().info('joint_state_aligner listo (escucha /joint_states)')

    def cb(self, msg: JointState):
        out = JointState()
        out.header = msg.header
        out.name   = list(msg.name)
        # aplica offset por nombre; si un nombre no está en el dict, lo deja igual
        out.position = [
            (p + self.offsets.get(name, 0.0)) for name, p in zip(msg.name, msg.position)
        ]
        out.velocity = list(msg.velocity)
        out.effort   = list(msg.effort)
        self.pub.publish(out)

def main():
    rclpy.init()
    node = JointStateAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
