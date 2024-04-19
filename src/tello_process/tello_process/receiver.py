import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class TelloSubscrbver(Node):

    def __init__(self):
        super().__init__('tello_image_reciever')
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.subscription  
        self.get_logger().info("Image subscriber node initialized")



    def image_callback(self, msg):
        self.get_logger().info("Received an image")
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Tello Image", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    # minimal_publisher = MinimalPublisher()
    tello_sub = TelloSubscrbver()

    rclpy.spin(tello_sub)
    tello_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()