import time
from siyi_sdk import SIYISDK
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
import math

class GimbalControlNode(Node):
    def __init__(self):
        super().__init__('gimbal_control_node')
        self.subscription = self.create_subscription(
            Point,
            'position_difference',
            self.listener_callback,
            10
        )
        self.cam = SIYISDK(server_ip="192.168.144.25", port=37260)
        self.initialize_gimbal()
        
        # 제어 파라미터 설정
        self.kp = 0.06 # 비례 이득
        self.ki = 0.001  # 적분 이득
        self.kd = 0.003  # 미분 이득
        self.prev_error_yaw = 0
        self.prev_error_pitch = 0
        self.integral_yaw = 0
        self.integral_pitch = 0

    def initialize_gimbal(self):
        """
        짐벌을 초기화하고 lock 모드로 설정한 후 초기 각도를 설정합니다.
        """
        # 짐벌 연결 확인 및 하드웨어 ID 요청
        if not self.cam.connect():
            print("짐벌과의 연결 실패")
            return
        time.sleep(1)
        
        self.cam.requestHardwareID()  # 하드웨어 각도 제한 정보 요청
        time.sleep(1)

        # Lock 모드 설정
        if not self.cam.requestLockMode():
            print("Lock 모드 설정 실패")
            return
        time.sleep(2)
        print("Current motion mode: ", self.cam._motionMode_msg.mode)
        
        # 초기 yaw, pitch 설정
        initial_yaw = 0
        initial_pitch = 0
        if not self.cam.requestSetAngles(initial_yaw, initial_pitch):
            print("초기 각도 설정 실패")
            return
        time.sleep(1)

        print("짐벌 초기화 완료 및 Lock 모드 설정 완료")

    def listener_callback(self, msg):
        """
        ROS 메시지로부터 x, y 차이 값을 받아서 짐벌의 각도를 업데이트합니다.
        """
        diff_x = msg.x
        diff_y = msg.y

        # PID 제어를 사용하여 yaw와 pitch 각도를 계산합니다.
        error_yaw = -diff_x
        error_pitch = -diff_y

        # 적분 항 계산 (적분 항 제한 없음)
        self.integral_yaw += error_yaw
        self.integral_pitch += error_pitch

        # 미분 항 계산
        derivative_yaw = error_yaw - self.prev_error_yaw
        derivative_pitch = error_pitch - self.prev_error_pitch

        # PID 제어를 사용하여 yaw와 pitch 각도 계산
        yaw_angle = (self.kp * error_yaw) + (self.ki * self.integral_yaw) + (self.kd * derivative_yaw)
        pitch_angle = (self.kp * error_pitch) + (self.ki * self.integral_pitch) + (self.kd * derivative_pitch)

        # 이전 에러 업데이트
        self.prev_error_yaw = error_yaw
        self.prev_error_pitch = error_pitch

        # 짐벌의 회전 각도를 업데이트하여 객체를 추적합니다.
        success = self.cam.requestSetAngles(-yaw_angle, -pitch_angle)
        if not success:
            print(f"짐벌 각도 업데이트 실패: Yaw={yaw_angle}, Pitch={pitch_angle}")
        else:
            print(f"짐벌 각도 업데이트: Yaw={yaw_angle}, Pitch={pitch_angle}")

def main(args=None):
    rclpy.init(args=args)

    # GimbalControlNode를 초기화합니다.
    gimbal_control_node = GimbalControlNode()

    try:
        rclpy.spin(gimbal_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        gimbal_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
