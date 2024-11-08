#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np
import os
import time

class MultiTrackerNode(Node):
    def __init__(self, video_path=None, playback_fps=30):
        super().__init__('multi_tracker')
        self.publisher = self.create_publisher(Point, 'position_difference', 10)
        
        # RTSP 스트림 경로를 지정한 경우
        rtsp_stream_url = "rtsp://192.168.144.25:8554/main.264"
        self.playback_fps = playback_fps
        self.frame_delay = 1 / self.playback_fps
        
        # 초기 스트림 연결 및 버퍼 설정
        self.initialize_stream(rtsp_stream_url)
        
        self.trackers = {}
        self.tracker_colors = {
            'CSRT': (0, 255, 0),
            'KCF': (255, 0, 0),
            'MedianFlow': (0, 0, 255),
            'MIL': (255, 255, 0),
            #'MOSSE': (0, 255, 255)
            # Disabled due to poor performance.
        }
        self.tracking = False
        
        # Get the frame dimensions
        _, frame = self.cap.read()
        if frame is not None:
            self.frame_height, self.frame_width = frame.shape[:2]
            self.frame_center_x = self.frame_width // 2
            self.frame_center_y = self.frame_height // 2
        else:
            self.get_logger().error('Could not read initial frame')
            raise IOError("Couldn't read initial frame from video source")

    def initialize_stream(self, rtsp_stream_url):
        # RTSP 스트림 초기화 및 버퍼 설정
        self.cap = cv2.VideoCapture(rtsp_stream_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)  # Set buffer size to 0 to reduce delay
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video source')
            raise IOError("Couldn't open video source")
        self.get_logger().info(f'Using RTSP stream: {rtsp_stream_url}')

    def select_roi(self, frame):
        roi = cv2.selectROI("Select ROI", frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select ROI")
        return roi

    def initialize_trackers(self, frame, roi):
        # Clear buffer by reading frames until the latest one
        for _ in range(10):
            self.cap.grab()  # Skip a few frames to reach the latest
        ret, latest_frame = self.cap.read()
        if not ret or latest_frame is None:
            self.get_logger().warn('Could not read frame after ROI selection, using previous frame')
            latest_frame = frame

        self.trackers = {
            'CSRT': cv2.TrackerCSRT_create(),
            #'KCF': cv2.TrackerKCF_create(),
            #'MedianFlow': cv2.TrackerMedianFlow_create(),
            #'MIL': cv2.TrackerMIL_create(),
            #'MOSSE': cv2.legacy.TrackerMOSSE_create()
            # Disabled due to poor performance.
        }
        for tracker in self.trackers.values():
            tracker.init(latest_frame, roi)
        self.tracking = True

    def track_object(self):
        while rclpy.ok():
            start_time = time.time()
            
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn('Could not read frame, attempting to reconnect...')
                self.initialize_stream("rtsp://192.168.144.25:8554/main.264")
                continue

            if self.tracking:
                success_flags = {}
                boxes = {}
                for name, tracker in self.trackers.items():
                    success, box = tracker.update(frame)
                    success_flags[name] = success
                    boxes[name] = box

                if any(success_flags.values()):
                    self.process_tracking(frame, boxes, success_flags)
                else:
                    self.get_logger().warn('All trackers failed')
                    self.tracking = False
                    cv2.putText(frame, "Tracking lost", (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(frame, f"Playback Speed: {self.playback_fps:.1f} fps", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow('Multi Tracker', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                roi = self.select_roi(frame)
                self.initialize_trackers(frame, roi)
            elif key == ord('p'):
                cv2.waitKey(-1)  # Wait until any key is pressed
            elif key == ord('+') or key == ord('='):
                self.playback_fps = min(self.playback_fps + 1, self.playback_fps)
                self.frame_delay = 1 / self.playback_fps
            elif key == ord('-'):
                self.playback_fps = max(self.playback_fps - 1, 1)
                self.frame_delay = 1 / self.playback_fps

            rclpy.spin_once(self, timeout_sec=0)
            
            # Control frame rate
            elapsed_time = time.time() - start_time
            sleep_time = max(self.frame_delay - elapsed_time, 0)
            time.sleep(sleep_time)

    def process_tracking(self, frame, boxes, success_flags):
        valid_boxes = []
        differences = {}
        for name, success in success_flags.items():
            if success:
                box = boxes[name]
                self.draw_box(frame, box, self.tracker_colors[name], name)
                valid_boxes.append(box)
                diff = self.calculate_difference(box)
                differences[name] = diff
                self.display_difference(frame, name, diff, self.tracker_colors[name])

        if valid_boxes:
            avg_box = np.mean(valid_boxes, axis=0)
            avg_diff = self.calculate_difference(avg_box)
            self.publish_difference(avg_diff)
            self.display_difference(frame, "Average", avg_diff, (255, 255, 255))

    def draw_box(self, frame, box, color, label):
        (x, y, w, h) = [int(v) for v in box]
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        roi_center_x, roi_center_y = x + w // 2, y + h // 2
        cv2.circle(frame, (roi_center_x, roi_center_y), 5, color, -1)
        
        # Draw frame center
        cv2.circle(frame, (self.frame_center_x, self.frame_center_y), 5, (255, 0, 0), -1)
        
        # Draw line from frame center to ROI center
        cv2.line(frame, (self.frame_center_x, self.frame_center_y),
                 (roi_center_x, roi_center_y), color, 2)

    def calculate_difference(self, box):
        (x, y, w, h) = box
        roi_center_x, roi_center_y = x + w // 2, y + h // 2
        diff_x = roi_center_x - self.frame_center_x
        diff_y = roi_center_y - self.frame_center_y
        return (diff_x, diff_y)

    def display_difference(self, frame, name, diff, color):
        diff_x, diff_y = diff
        y_position = 60 + 30 * (list(self.tracker_colors.keys()).index(name) if name != "Average" else len(self.tracker_colors))
        cv2.putText(frame, f"{name} Diff: ({diff_x:.1f}, {diff_y:.1f})", (10, y_position),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    def publish_difference(self, diff):
        msg = Point()
        msg.x = float(diff[0])
        msg.y = float(diff[1])
        msg.z = 0.0
        self.publisher.publish(msg)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    playback_fps = float(input("Enter desired playback FPS (default is 30): ") or 30)
    multi_tracker = MultiTrackerNode(playback_fps=playback_fps)
    try:
        multi_tracker.track_object()
    except KeyboardInterrupt:
        pass
    finally:
        multi_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
