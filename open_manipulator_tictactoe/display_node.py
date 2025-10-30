#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DisplayNode(Node):
    def __init__(self):
        super().__init__("DisplayNode")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

    def image_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(1)
            #self.get_logger().info("Image displayed successfully.")

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node=DisplayNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()