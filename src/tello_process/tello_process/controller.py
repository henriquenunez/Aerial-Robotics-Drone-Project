import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32,Float32MultiArray
import cv2
from rclpy.action import ActionClient
from tello_msgs.srv import TelloAction
from geometry_msgs.msg import Twist
import random
import time
import math

class TelloController(Node):

    def __init__(self):
        super().__init__('tello_image_reciever')
        self.get_logger().info("Controller node initialized")
        self.go_through = 0

        self.frame_coord_sub = self.create_subscription(
            Float32MultiArray,
            '/frame_coord',
            self.frame_coord_cb,
            rclpy.qos.qos_profile_sensor_data
        )
        
        self.fps_sub = self.create_subscription(
            Float32,
            '/fps',
            self.fps_cb,
            rclpy.qos.qos_profile_sensor_data
        )
        self.fps = 15

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

        # Controller parameters
        self.poi_thresh = 0.1
        self.centralized_frame = False

    def fps_cb(self, data):
        self.fps = data.data

    def frame_coord_cb(self, data):
        self.get_logger().info(f'Frame data: {data.data}, FPS: {self.fps}')

        self.speed = 0.15 * (self.fps / 15) # 15 FPS was the base of our tests

        tgt_x, tgt_y, detected_stop, conf_stop = data.data

        if tgt_x > -1000 or tgt_y > -1000:
            self.has_poi = True
        else:
            self.has_poi = False

        self.stop_sign = detected_stop == 1.0
        
        if self.stop_sign and conf_stop > 70000:
            self.stop_sign_close = True
        else:
            self.stop_sign_close = False

        self.target = (tgt_x, tgt_y)

        # else:
        #     if detected_stop == 1.0 and conf_stop > 23000:
        #         self.get_logger().info('Landing Tello')
        #         self.tello_req.cmd = 'land'
        #         takeoff_future = self.tello_client.call_async(self.tello_req)
        #         rclpy.spin(self, takeoff_future)
        #         self.get_logger().info('Tello Landed')
        #     else:
        #         twist_msg.linear.x = 0.05
        #         twist_msg.linear.y = 0.00
        #         twist_msg.linear.z = 0.00
         
        # self.get_logger().info(f'Sending Twist: {twist_msg}')
        # return twist_msg

        twist_msg = self.control()
        
        if twist_msg is not None:
            self.tello_vel_pub.publish(twist_msg)

    def control(self):

        twist_msg = Twist()

        # TODO: check if it already starts at 0
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        if self.stop_sign_close:
            # TODO: land
            self.get_logger().info('Landing Tello')
            self.tello_req.cmd = 'land'
            takeoff_future = self.tello_client.call_async(self.tello_req)
            rclpy.spin(self, takeoff_future)
            self.get_logger().info('Tello Landed')
            return None

        if not self.has_poi:

            if self.centralized_frame: # We were in a frame rn
                twist_msg.linear.x = self.speed
                return twist_msg

            pass # TODO: start a turning strategy with state machine

        else:
            tgt_x, tgt_y = self.target
            okX = okY = False

            # Proportional controller
            speed_z = self.speed * abs(tgt_y)
            speed_ang = self.speed * abs(tgt_x)

            # Increase and decrease height
            if tgt_y > self.poi_thresh:
                self.get_logger().info('Go down')

                twist_msg.linear.z = -speed_z
                twist_msg.linear.x = 0.0 
            elif tgt_y < -self.poi_thresh:
                self.get_logger().info('Go up')

                twist_msg.linear.z = speed_z
                twist_msg.linear.x = 0.0 
            else:
                okX = True
    
            # Spin horizontally (z axis) to adjust yaw
            if tgt_x > self.poi_thresh:
                self.get_logger().info('Go left')

                twist_msg.angular.z = -speed_ang
                twist_msg.linear.x = 0.0 
            elif tgt_x < -self.poi_thresh:

                self.get_logger().info('Go right')

                twist_msg.angular.z = speed_ang
                twist_msg.linear.x = 0.0 
            else:
                okY = True

            if okX and okY:
                self.get_logger().info('OK')
                twist_msg.linear.x = self.speed
                self.centralized_frame = True
        
        return twist_msg

 
def main(args=None):
    rclpy.init(args=args)

    tello_controll = TelloController()

    rclpy.spin(tello_controll)
    tello_controll.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

