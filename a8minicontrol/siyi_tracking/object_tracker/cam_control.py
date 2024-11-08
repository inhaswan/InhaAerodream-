#!/usr/bin/env python3

# 카메라를 lock 모드로 설정하고 yaw, pitch, roll을 0도로 설정하고 ROI가 카메라 중심에 오도록 제어하는 코드

import rclpy
from rclpy.node import Node
from siyi_control import SIYIControl
from geometry_msgs.msg import Point
import time

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control')
        self.siyi_control = SIYIControl()
        self.subscription = self.create_subscription(
            Point,
            'position_difference',
            self.listener_callback,
            10
        )
        self.get_logger().info('Camera control node initialized')

        # Lock 모드를 설정하고 현재 모션 모드를 출력
        if self.siyi_control.cam.requestLockMode():
            self.get_logger().info('Initial motion mode set to lock.')
            time.sleep(2)  # Lock 모드 설정 후 안정화 시간 대기
        else:
            self.get_logger().error('Failed to set initial motion mode to lock.')
            return

        # 초기 yaw, pitch, roll 각도를 0도로 설정
        self.siyi_control.set_offset(yaw_off=0.0, pitch_offset=0.0, power=1)
        self.get_logger().info('Yaw, pitch, and roll set to 0 degrees.')

    def listener_callback(self, msg):
        # ROI의 위치 차이를 수신하고 카메라를 중앙으로 이동
        self.get_logger().info(f'Received position difference: x={msg.x}, y={msg.y}')
        self.update_camera(msg.x, msg.y)

    def update_camera(self, diff_x, diff_y):
        # ROI가 카메라 중심에 오도록 제어
        power = 1  # 카메라 이동에 필요한 파워 값
        yaw_offset = -diff_x * power 
        pitch_offset = -diff_y * power

        # 카메라 위치 조정 명령
        self.siyi_control.set_offset(yaw_off=yaw_offset, pitch_offset=pitch_offset, power=power)
        self.get_logger().info(f'Camera adjusted with yaw offset: {yaw_offset}, pitch offset: {pitch_offset}')

        # 피드백 루프: 카메라 위치가 목표에 도달할 때까지 반복적으로 조정
        time.sleep(1)  # 이동 후 안정화 시간 대기
        attitude_msg = getattr(self.siyi_control.cam, '_attitude_msg', None)
        if attitude_msg:
            yaw, pitch = attitude_msg.yaw, attitude_msg.pitch
            if abs(diff_x) > 5 or abs(diff_y) > 5:  # 허용 오차 범위 내에 도달할 때까지 반복
                self.update_camera(diff_x, diff_y)

    def __del__(self):
        # 노드 종료 시 로그 메시지 출력
        self.get_logger().info('Shutting down camera control node')


def main(args=None):
    # ROS 2 초기화 및 CameraControlNode 실행
    rclpy.init(args=args)
    camera_control_node = CameraControlNode()
    try:
        rclpy.spin(camera_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
