import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

LOWER_BROWN = np.array([74, 41, 48])
UPPER_BROWN = np.array([128, 204, 206])

LOWER_GREEN = np.array([40, 40, 40])
UPPER_GREEN = np.array([70, 255, 255])

LOWER_RED = np.array([0, 80, 50])
UPPER_RED = np.array([50, 255, 255])


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

        #UNCOMMENT FOR DEBUGGING TODO: CLEAN IT!#
        # cv2.namedWindow('Threshold Adjustments')
        # self.lower_bound = LOWER_BROWN
        # self.upper_bound = UPPER_BROWN
        # cv2.createTrackbar('Lower H', 'Threshold Adjustments',
        #                     0, 255, self.update_threshold)
        # cv2.createTrackbar('Upper H', 'Threshold Adjustments',
        #                     0, 255, self.update_threshold)

        # cv2.createTrackbar('Lower S', 'Threshold Adjustments',
        #                     0, 255, self.update_threshold)
        # cv2.createTrackbar('Upper S', 'Threshold Adjustments',
        #                     0, 255, self.update_threshold)

        # cv2.createTrackbar('Lower V', 'Threshold Adjustments',
        #                     0, 255, self.update_threshold)
        # cv2.createTrackbar('Upper V', 'Threshold Adjustments',
        #                     0, 255, self.update_threshold)

        # cv2.setTrackbarPos('Upper H', 'Threshold Adjustments', 255)
        # cv2.setTrackbarPos('Upper S', 'Threshold Adjustments', 255)
        # cv2.setTrackbarPos('Upper V', 'Threshold Adjustments', 255)

        self.frame_coord_pub = self.create_publisher(Float32MultiArray, '/frame_coord', 10)
        self.get_logger().info("Image subscriber node initialized")

    def update_threshold(self, x):
        pass
        #UNCOMMENT FOR DEBUGGING TODO: CLEAN IT!#
        # lower_h = cv2.getTrackbarPos('Lower H', 'Threshold Adjustments')
        # upper_h = cv2.getTrackbarPos('Upper H', 'Threshold Adjustments')

        # lower_s = cv2.getTrackbarPos('Lower S', 'Threshold Adjustments')
        # upper_s = cv2.getTrackbarPos('Upper S', 'Threshold Adjustments')

        # lower_v = cv2.getTrackbarPos('Lower V', 'Threshold Adjustments')
        # upper_v = cv2.getTrackbarPos('Upper V', 'Threshold Adjustments')

        # self.lower_bound = np.array([lower_h, lower_s, lower_v])
        # self.upper_bound = np.array([upper_h, upper_s, upper_v])

    def image_callback(self, msg):

        # self.get_logger().info("Received an image")
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.process_frame(cv_image)

    def process_frame(self, image):

        image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask_brown = cv2.inRange(hsv, LOWER_BROWN, UPPER_BROWN)
        mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        mask_red = cv2.inRange(hsv, LOWER_RED, UPPER_RED)

        mask_combined = cv2.bitwise_or(mask_brown, mask_green)
        mask = cv2.bitwise_or(mask_combined, mask_red)

        contours, hierarchy = cv2.findContours(mask,
                                                cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            inner_contours = []
            for i, contour in enumerate(contours):
                _, _, child_idx, parent_idx = hierarchy[0][i]
                
                if child_idx == -1:
                    inner_contours.append(contour)
                    cv2.drawContours(image, [contour], -1, (0, 255, 255), 2)

            areas = [cv2.contourArea(cnt) for cnt in inner_contours]
            larger_inner_contour = contours[np.argmax(areas)]
            image = cv2.drawContours(image,[larger_inner_contour],
                                                    -1, (0, 0, 255), 2)


            for i, contour in enumerate(inner_contours):
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
                average_point = (int(avg_x), int(avg_y))
                if i == np.argmax(areas):
                    self.final_average_point = (int(avg_x), int(avg_y))
                    cv2.circle(image, average_point, 5, (255, 0, 0), -1)
                else:
                    cv2.circle(image, average_point, 5, (0, 0, 255), -1)

            cv2.drawContours(image, contours, -1, (255, 100, 100), 2)


        cv2.imshow("res", mask)
        cv2.imshow("Tello Image", image)
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

