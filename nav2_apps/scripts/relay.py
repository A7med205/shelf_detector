#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class TfRelayNode(Node):
    def __init__(self):
        super().__init__('tf_relay_node')

        # QoS for subscribing to /tf 
        # Many tf publishers use TRANSIENT_LOCAL durability so late joiners get transforms.
        # We'll match that here.
        tf_sub_qos = QoSProfile(depth=10)
        tf_sub_qos.durability = QoSDurabilityPolicy.VOLATILE
        tf_sub_qos.reliability = QoSReliabilityPolicy.RELIABLE

        # QoS for publishing /tf_relay 
        # Use a more lenient QoS that might be compatible with rosbridge (e.g., VOLATILE)
        tf_pub_qos = QoSProfile(depth=10)
        tf_pub_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        tf_pub_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            tf_sub_qos
        )

        self.tf_publisher = self.create_publisher(
            TFMessage,
            '/tf_relay',
            tf_pub_qos
        )

        self.get_logger().info('TF Relay Node started, relaying /tf -> /tf_relay with adjusted QoS.')

    def tf_callback(self, msg):
        # Simply relay the message to /tf_relay
        self.tf_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
