import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

LOWER_GREEN = np.array([40, 40, 40])
UPPER_GREEN = np.array([70, 255, 255])

class TelloSubscriber(Node):

    def __init__(self):
        super().__init__('tello_image_reciever')
        self.final_average_point = []

        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.frame_coord_pub = self.create_publisher(Float32MultiArray, '/frame_coord', 10)
        self.get_logger().info("Image subscriber node initialized")

    def image_callback(self, msg):

        # self.get_logger().info("Received an image")
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.process_frame(cv_image)

    def process_frame(self, image):

        image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

        contours, hierarchy = cv2.findContours(mask,
                                                cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        
        inner_contours = []
        for i, contour in enumerate(contours):
            _, _, child_idx, parent_idx = hierarchy[0][i]
            
            if child_idx == -1:
                inner_contours.append(contour)
                cv2.drawContours(image, [contour], -1, (0, 255, 255), 2)
        for contour in inner_contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            sq_points = []
            for point in approx:
                x, y = point.ravel()
                sq_points.append([x, y])
                cv2.circle(image, (x, y), 5, (0, 0, 255), -1)

            x_coords = [point[0] for point in sq_points]
            y_coords = [point[1] for point in sq_points]

            avg_x = sum(x_coords) / len(x_coords)
            avg_y = sum(y_coords) / len(y_coords)
            self.final_average_point = (int(avg_x), int(avg_y))
            cv2.circle(image, self.final_average_point, 5, (255, 150, 255), -1)


        cv2.drawContours(image, contours, -1, (255, 100, 100), 2)
        cv2.imshow("Tello Image", image)
        cv2.imshow("res", mask)
        cv2.waitKey(1)
        
        if self.final_average_point:
            array = Float32MultiArray(data=[self.final_average_point[0],
                                         self.final_average_point[1]])
            self.frame_coord_pub.publish(array)

def main(args=None):
    rclpy.init(args=args)

    tello_sub = TelloSubscriber()

    rclpy.spin(tello_sub)
    tello_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

