import time
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from geometry_msgs.msg import PoseStamped 

from six_dof_arm_perception import aruco_detect as ad # your image-based ArUco functions

ENCODING = 'bgr8'              # desired encoding, easy to change
STATUS_PERIOD_S = 3.0          # how often to print the camera status


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        # ArUco library used
        self.aruco_dict, self.aruco_params = ad.setup_detector()

        # Reference marker IDs and which were seen last frame
        self.marker_ids = list(ad.ARUCO_WORLD_POSITIONS.keys())
        self.prev_detected = [False] * len(self.marker_ids)
        self.last_status_time = 0.0

        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)

        self.pose_publisher = self.create_publisher(PoseStamped, 'camera_pose', 10)
        self.br = CvBridge()

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding=ENCODING)

        # Detect reference markers and get the averaged camera pose
        camera_matrix = ad.make_camera_matrix(current_frame)
        T_WC, detected_ids = ad.detect_frame(
            current_frame, camera_matrix, self.aruco_dict, self.aruco_params)

        # Log if the set of detected reference markers changes
        detected = [m in detected_ids for m in self.marker_ids]
        if detected != self.prev_detected:
            self.get_logger().info(f'Reference markers detected: {detected_ids}')
            self.prev_detected = detected
        
        if T_WC is not None:
            msg = PoseStamped()
            msg.header.stamp = data.header.stamp
            msg.header.frame_id = 'world'

            x, y, z = T_WC[:3, 3]
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = float(z)

            qx, qy, qz, qw = ad.rotation_to_quaternion(T_WC[:3, :3])
            msg.pose.orientation.x = float(qx)
            msg.pose.orientation.y = float(qy)
            msg.pose.orientation.z = float(qz)
            msg.pose.orientation.w = float(qw)

            self.pose_publisher.publish(msg)
        # Print the status every few seconds
        now = time.time()
        if now - self.last_status_time > STATUS_PERIOD_S:
            if T_WC is not None:
                self.get_logger().info(f'Camera at {np.round(T_WC[:3, 3] * 100, 1)} cm')
            else:
                self.get_logger().info('No reference markers visible')
            self.last_status_time = now

        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()