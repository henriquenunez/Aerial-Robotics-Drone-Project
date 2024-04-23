import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import cv2
from rclpy.action import ActionClient
from tello_msgs.srv import TelloAction
from geometry_msgs.msg import Twist
import random
import time

class TelloController(Node):

    def __init__(self):
        super().__init__('tello_image_reciever')
        self.get_logger().info("Controller node initialized")
        self.frame_coord_sub = self.create_subscription(
            Float32MultiArray,
            '/frame_coord',
            self.frame_coord_cb,
            rclpy.qos.qos_profile_sensor_data
        )
        self.tello_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

        self.tello_client = self.create_client(TelloAction, '/drone1/tello_action')
        time.sleep(3)
        while not self.tello_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for TelloAction service')

        self.tello_req = TelloAction.Request()
        self.tello_req.cmd = 'takeoff'
        takeoff_future = self.tello_client.call_async(self.tello_req)
        rclpy.spin_until_future_complete(self, takeoff_future)
        self.get_logger().info('took off!')

    def frame_coord_cb(self, data):
        self.get_logger().info(f'Got: {data.data}')

        twist_msg = self.control(data.data)
        
        self.tello_vel_pub.publish(twist_msg)

    def control(self, frame_target_coord):
        twist_msg = Twist()
        tgt_x, tgt_y = frame_target_coord

        speed = 0.01
        
        
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        twist_msg.linear.x = speed/5
        
        if tgt_x > 0.1:
            twist_msg.linear.z = -speed
 
        if tgt_x < -0.1:
            twist_msg.linear.z = speed
 
        if tgt_y > 0.1:
            twist_msg.linear.y = -speed
 
        if tgt_y < -0.1:
            twist_msg.linear.y = speed

        return twist_msg

def main(args=None):
    rclpy.init(args=args)

    tello_controll = TelloController()

    rclpy.spin(tello_controll)
    tello_controll.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
