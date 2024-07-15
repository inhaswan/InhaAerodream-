import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import yaml

class ArUcoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(Float32MultiArray, '/vector', qos_profile)
        self.pose_publisher = self.create_publisher(PoseStamped, 'estimated_pose', 10)
        self.cap = cv2.VideoCapture(0)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.vector = Float32MultiArray()
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz timer
        self.marker_i = {0, 1}
        self.marker_f = {45}
        self.marker_current = self.marker_i

        self.last_altitude = 0
        self.altitude_compensation = 0.08  # 8cm compensation

        # Load camera calibration
        self.load_calibration('/home/seawhan/test/camera_calibration.yaml')

    def load_calibration(self, file_path):
        with open(file_path, 'r') as file:
            calibration_data = yaml.safe_load(file)
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])

    def get_marker_size(self, marker_id, altitude):
        if altitude < 0.8:  # Below 80 cm
            if marker_id == 45:
                return 0.1  # 10 cm
        else:  # Above or equal to 80 cm
            if marker_id in [0, 1]:
                return 0.17  # 17 cm
        return None  # Invalid marker for current altitude

    def calculate_altitude(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        z_camera = R.T[2]
        cos_theta = np.dot(z_camera, [0, 0, 1])
        theta = np.arccos(cos_theta)
        vertical_distance = abs(tvec[0][2] / np.cos(theta))
        compensated_altitude = vertical_distance - self.altitude_compensation
        return max(compensated_altitude, 0)  # Ensure altitude is not negative

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to grab frame')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        if ids is not None:
            ids = ids.flatten()
            marker_centers = []

            for (marker_corner, marker_id) in zip(corners, ids):
                if marker_id in self.marker_current:
                    marker_size = self.get_marker_size(marker_id, self.last_altitude)
                    if marker_size is not None:
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([marker_corner], marker_size, self.camera_matrix, self.dist_coeffs)
                        altitude = self.calculate_altitude(rvecs[0], tvecs[0])
                        self.last_altitude = altitude

                        # Publish PoseStamped message
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = 'camera_frame'
                        pose_msg.pose.position.z = altitude
                        self.pose_publisher.publish(pose_msg)

                        # Calculate marker center and vector
                        corners = marker_corner.reshape((4, 2))
                        top_left, top_right, bottom_right, bottom_left = corners
                        bbox_center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                        bbox_center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                        vector_x = bbox_center_x - frame_center_x
                        vector_y = bbox_center_y - frame_center_y
                        marker_centers.append((vector_x, vector_y))

                        # Draw marker and info on frame
                        cv2.aruco.drawDetectedMarkers(frame, [marker_corner], np.array([marker_id]))
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.1)
                        cv2.putText(frame, f'Altitude: {altitude:.2f} m, ID: {marker_id}, Size: {marker_size*100:.0f}cm', 
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            #draw arrow to average marker center
            if marker_centers:
                avg_vector_x = sum([center[0] for center in marker_centers]) / len(marker_centers)
                avg_vector_y = sum([center[1] for center in marker_centers]) / len(marker_centers)
                self.vector.data = [float(avg_vector_x), float(avg_vector_y)]

                cv2.arrowedLine(frame, (frame_center_x, frame_center_y), 
                                (int(avg_vector_x + frame_center_x), int(avg_vector_y + frame_center_y)), 
                                (0, 0, 255), 2)
                self.publisher.publish(self.vector)
                self.get_logger().info(f"Published average vector (x, y): ({avg_vector_x}, {avg_vector_y})")

            # Switch marker sets based on altitude
            if self.last_altitude < 0.8 and self.marker_current == self.marker_i:
                self.marker_current = self.marker_f
                self.get_logger().info(f"Switching to secondary marker IDs: {self.marker_f}")
            elif self.last_altitude >= 0.8 and self.marker_current == self.marker_f:
                self.marker_current = self.marker_i
                self.get_logger().info(f"Switching to initial marker IDs: {self.marker_i}")

        cv2.imshow('ArUco Marker Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_detector = ArUcoMarkerDetector()
    rclpy.spin(aruco_marker_detector)
    aruco_marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()