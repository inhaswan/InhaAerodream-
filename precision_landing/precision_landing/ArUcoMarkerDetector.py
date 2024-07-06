import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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
        self.cap = cv2.VideoCapture(0)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.vector = Float32MultiArray()
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz timer
        self.marker_i = {0, 1}
        self.marker_f = {45}
        self.marker_current = self.marker_i
        self.marker_f_length = 0.05

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to grab frame')
            return

        (corners, ids, _) = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        if ids is not None:
            ids = ids.flatten()

            if self.marker_current == self.marker_i:
                detected_i_markers = [marker_id for marker_id in ids if marker_id in self.marker_i]
                if len(detected_i_markers) >= 2:
                    marker_centers = []
                    for (marker_corner, marker_id) in zip(corners, ids):
                        if marker_id in self.marker_i:
                            corners = marker_corner.reshape((4, 2))
                            corners = [(int(corner[0]), int(corner[1])) for corner in corners]
                            (top_left, top_right, bottom_right, bottom_left) = corners

                            ###########################################################
                            # 거리 측정을 위한 코드 (cv2.aruco.estimatePoseSingleMarkers)
                            distance = 0
                            ###########################################################

                            if distance < 0.1:
                                self.marker_current = self.marker_f
                                marker_centers = []
                                self.get_logger().info(f"Switching to secondary marker IDs: {self.marker_f}")
                                continue

                            bbox_center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                            bbox_center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                            vector_x = bbox_center_x - frame_center_x
                            vector_y = bbox_center_y - frame_center_y
                            marker_centers.append((vector_x, vector_y))

                            cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                            cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                            cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                            cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
                            cv2.circle(frame, (bbox_center_x, bbox_center_y), 4, (0, 0, 255), -1)
                            cv2.putText(frame, str(marker_id), (top_left[0], top_left[1] - 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    if marker_centers:
                        avg_vector_x = sum([center[0] for center in marker_centers]) / len(marker_centers)
                        avg_vector_y = sum([center[1] for center in marker_centers]) / len(marker_centers)
                        self.vector.data = [float(avg_vector_x), float(avg_vector_y)]

                        cv2.arrowedLine(frame, (frame_center_x, frame_center_y), (int(avg_vector_x + frame_center_x), int(avg_vector_y + frame_center_y)), (0, 0, 255), 2)
                        self.publisher.publish(self.vector)
                        self.get_logger().info(f"Published average vector (x, y) for markers: ({avg_vector_x}, {avg_vector_y})")
            
            elif self.marker_current == self.marker_f:
                marker_centers = []
                for (marker_corner, marker_id) in zip(corners, ids):
                    if marker_id in self.marker_current:
                        corners = marker_corner.reshape((4, 2))
                        corners = [(int(corner[0]), int(corner[1])) for corner in corners]
                        (top_left, top_right, bottom_right, bottom_left) = corners

                        ###########################################################
                        # 거리 측정을 위한 코드 (cv2.aruco.estimatePoseSingleMarkers)
                        distance = 0
                        ###########################################################

                        if distance > 0.1:
                            self.marker_current = self.marker_i
                            marker_centers = []
                            self.get_logger().info(f"Switching to initial marker IDs: {self.marker_i}")
                            continue

                        bbox_center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                        bbox_center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                        vector_x = bbox_center_x - frame_center_x
                        vector_y = bbox_center_y - frame_center_y
                        marker_centers.append((vector_x, vector_y))

                        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
                        cv2.circle(frame, (bbox_center_x, bbox_center_y), 4, (0, 0, 255), -1)
                        cv2.putText(frame, str(marker_id), (top_left[0], top_left[1] - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                if marker_centers:
                    avg_vector_x = sum([center[0] for center in marker_centers]) / len(marker_centers)
                    avg_vector_y = sum([center[1] for center in marker_centers]) / len(marker_centers)
                    self.vector.data = [float(avg_vector_x), float(avg_vector_y)]

                    # Draw arrowed line from frame center to average vector point
                    cv2.arrowedLine(frame, (frame_center_x, frame_center_y), (int(avg_vector_x + frame_center_x), int(avg_vector_y + frame_center_y)), (0, 0, 255), 2)

                    self.publisher.publish(self.vector)
                    self.get_logger().info(f"Published average vector (x, y) for marker 45: ({avg_vector_x}, {avg_vector_y})")

        cv2.imshow('frame', frame)
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
    
