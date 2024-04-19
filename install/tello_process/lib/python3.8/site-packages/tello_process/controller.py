import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class TelloController(Node):

    def __init__(self):
        pass



def main(args=None):
    rclpy.init(args=args)

    tello_controll = TelloController()

    rclpy.spin(tello_controll)
    tello_controll.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()