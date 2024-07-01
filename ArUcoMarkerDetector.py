import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ArUcoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.publisher = self.create_publisher(Float32MultiArray, '/aruco_marker_vector', 10)
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz timer

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to grab frame')
            return

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()

        (corners, ids, _) = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        if ids is not None:
            ids = ids.flatten()
            for (marker_corner, marker_id) in zip(corners, ids):
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                bbox_center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                bbox_center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(frame, (bbox_center_x, bbox_center_y), 4, (0, 0, 255), -1)
                cv2.putText(frame, str(marker_id), (top_left[0], top_left[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                vector_x = bbox_center_x - frame_center_x
                vector_y = bbox_center_y - frame_center_y

                vector_msg = Float32MultiArray()
                vector_msg.data = [float(vector_x), float(vector_y)]
                self.publisher.publish(vector_msg)

                self.get_logger().info(f"Published vector (x, y) for marker {marker_id}: ({vector_x}, {vector_y})")

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
