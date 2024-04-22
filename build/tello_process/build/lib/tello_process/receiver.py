import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import cv2

class TelloSubscriber(Node):

    def __init__(self):
        super().__init__('tello_image_reciever')
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.frame_coord_pub = self.create_publisher(Float32MultiArray, '/frame_coord', 10)
        self.get_logger().info("Image subscriber node initialized")

    def image_callback(self, msg):
        #self.get_logger().info("Received an image")
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Tello Image", cv_image)
        cv2.waitKey(1)
        self.process_frame(cv_image)

    def process_frame(self, image):
        # TODO: find the center coordinates for the frontmost frame
        array = Float32MultiArray(data=[20.0, 20.0, 320.0, 20.0])
        self.frame_coord_pub.publish(array)
        #self.get_logger().info("Processed frame")

def main(args=None):
    rclpy.init(args=args)

    tello_sub = TelloSubscriber()

    rclpy.spin(tello_sub)
    tello_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

