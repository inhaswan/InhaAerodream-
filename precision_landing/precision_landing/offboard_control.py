import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
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

        self.publisher_vector = self.create_publisher(Float32MultiArray, '/vector', qos_profile)
        self.publisher_stop = self.create_publisher(String, '/disarm', qos_profile)

        self.cap = cv2.VideoCapture(0)
        self.aruco_dicts = [cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)]
        self.aruco_dict_index = 0
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.vector = Float32MultiArray()
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to grab frame')
            return

        aruco_dict = self.aruco_dicts[self.aruco_dict_index]

        (corners, ids, _) = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=self.aruco_params)
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        if ids is not None:
            ids = ids.flatten()
            for (marker_corner, marker_id) in zip(corners, ids):
                corners = marker_corner.reshape((4, 2))
                corners = [(int(corner[0]), int(corner[1])) for corner in corners]
                (top_left, top_right, bottom_right, bottom_left) = corners
                bbox_center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                bbox_center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                vector_x = bbox_center_x - frame_center_x
                vector_y = bbox_center_y - frame_center_y
                self.vector.data = [float(vector_x), float(vector_y)]

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
                cv2.circle(frame, (bbox_center_x, bbox_center_y), 4, (0, 0, 255), -1)
                cv2.putText(frame, str(marker_id), (top_left[0], top_left[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.publisher_vector.publish(self.vector)
                self.get_logger().info(f"Published vector (x, y) for marker {marker_id}: ({vector_x}, {vector_y})")

                # 4x4 마커의 면적이 일정 크기 이상이면 'stop' 메시지 발행
                if aruco_dict == self.aruco_dicts[0]:  # 4x4 마커일 경우
                    area = cv2.contourArea(marker_corner)
                    if area > 2000: 
                        stop_msg = String()
                        stop_msg.data = 'stop'
                        self.publisher_stop.publish(stop_msg)
                        self.get_logger().warning("Published 'disarm' message.")

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

        self.aruco_dict_index = (self.aruco_dict_index + 1) % len(self.aruco_dicts)

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_detector = ArUcoMarkerDetector()
    rclpy.spin(aruco_marker_detector)
    aruco_marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
