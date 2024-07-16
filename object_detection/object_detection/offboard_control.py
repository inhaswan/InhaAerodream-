#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleOdometry, VehicleCommandAck
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np

class OffboardControl(Node):
    def __init__(self):
        super().__init__("px4_offboard_control")
        # px4와 통신하기 위한 qos 설정
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST,
                                 depth=1)

        self.offboard_control_mode_publisher = self.create_publisher(
                OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
                TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        # 오프보드 컨트롤 모드, 목표 위치 및 자세, 드론 명령 publish

        self.vehicle_local_position_subscriber = self.create_subscription(
                VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
                VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_odometry_subscriber = self.create_subscription(
                VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_command_ack_subscriber = self.create_subscription(
                VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, qos_profile)
        # 드론의 현재 위치, 상태, Odometry, Command Check를 위한 CommandAck subscribe 하기

        self.farthest_corner_subscriber = self.create_subscription(
            Point, 'farthest_corner', self.farthest_corner_callback, qos_profile)
        self.center_point_subscriber = self.create_subscription(
            Point, 'center_point', self.center_point_callback, qos_profile)
        self.distance_subscriber = self.create_subscription(
            Float64, 'distance', self.distance_callback, qos_profile)
        # 장애물의 최외각점, 중심점, 최외각점-중심점 사이의 최대 거리 subscribe 하기

        self.offboard_setpoint_counter = 0 # 몇번 offboard control 했는지 세는 변수
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_command_ack = VehicleCommandAck()
        # PX4 제어에 필요한 각종 변수들 선언

        self.farthest_corner = Point()
        self.center_point = Point()
        self.distance = Float64()
        # 장애물 회피에 필요한 각종 변수들 선언

        self.vehicle_vel_x = self.vehicle_odometry.velocity

        self.takeoff_height = -5.0
        # 테스트용 이륙 고도(변경 가능)

        self.timer = self.create_timer(0.1, self.timer_callback)
        # 0.1초마다 한번씩 오프보드 컨트롤 실행하기

        self.taken_off = False # 이륙 여부 플래그
        self.first_setpoint = False # 첫번째 경로점 도착 여부 플래그
        self.second_setpoint = False # 두번째 경로점 도착 여부 플래그
        self.landed = False # 착륙할지 결정하기 위한 플래그
        self.obstacle_detection = False # 장애물을 탐지했는지 보기 위한 플래그
        self.obstacle_avoidance = False # 장애물을 회피했는지 보기 위한 플래그

    def vehicle_local_position_callback(self, vehicle_local_position):
        # 드론의 현재 위치 저장
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        # 드론의 현재 상태 저장
        self.vehicle_status = vehicle_status

    def vehicle_odometry_callback(self, vehicle_odometry):
        # 드론의 현재 odometry 저장
        self.vehicle_odometry = vehicle_odometry

    def vehicle_command_ack_callback(self, vehicle_command_ack):
        # 드론의 현재 command_ack 저장
        self.vehicle_command_ack = vehicle_command_ack

    def farthest_corner_callback(self, farthest_corner):
        # 최외각점 저장
        self.farthest_corner = farthest_corner

    def center_point_callback(self, center_point):
        # 장애물 중심점 저장
        self.center_point = center_point

    def distance_callback(self, distance):
        # 최외각점까지의 거리를 저장
        self.distance = distance

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        # 이륙하기 위해서만 사용할거임
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx, vy, vz):
        """Publish a velocity setpoint."""
        # 안쓰는 값들 nan으로 선언안하면, 속도 제어 제대로 안먹힘
        msg = TrajectorySetpoint()
        msg.position = [np.nan, np.nan, np.nan]
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [np.nan, np.nan, np.nan]
        msg.jerk = [np.nan, np.nan, np.nan]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def calculate_direction_vector(self, current_x, current_y, current_z, target_x, target_y, target_z, speed=0.2):
        # 목표지점까지 방향 벡터를 계산하고, 원하는 속도에 곱해서 이동한다.
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z

        # 방향 벡터를 구하기 위해서 normalize
        norm_xyz = math.sqrt(dx**2 + dy**2 + dz**2)
        if norm_xyz > 0.0:
            vx = speed * dx / norm_xyz # 원하는 속도에 방향 벡터를 곱함
            vy = speed * dy / norm_xyz
            vz = speed * dz / norm_xyz
        else:
            vx, vy, vz = 0.0, 0.0, 0.0 # 만약 그런거 없으면 속도 제어 안함
        return vx, vy, vz

    def move_towards_target(self, target_x, target_y, target_z):
        # 목표지점까지 이동하기 위한 속도 벡터 구하기
        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y
        current_z = self.vehicle_local_position.z

        # Calculate direction vector towards the target
        vx, vy, vz = self.calculate_direction_vector(current_x, current_y, current_z, target_x, target_y, target_z)

        return vx, vy, vz

    def calculate_avoidance_vector(self, current_x, current_y, target_x, target_y, speed = 0.2):
        # 장애물과 내 위치를 계산해서 회피하는 방향 벡터를 계산함
        # 여기서 장애물을 회피하기 위해, 지속적으로 장애물에 접하는 수직 벡터를 생성함
        dx = target_x - current_x
        dy = target_y - current_y
        norm_xy = math.sqrt(dx**2 + dy**2)
        if norm_xy > 0.0:
            vx = speed * -dy / norm_xy # 그래서 dx, dy 위치를 바꾸고 -를 붙여줌
            vy = speed * dx / norm_xy
        else:
            vx, vy = 0.0, 0.0
        return vx, vy

    def avoid_from_target(self, target_x, target_y):
        """Calculate and return avoidance vector away from the specified target position."""
        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y

        # Calculate avoidance vector away from the target
        vx, vy = self.calculate_avoidance_vector(current_x, current_y, target_x, target_y)

        return vx, vy

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.taken_off is False:
            print("publishing takeoff")
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            if self.vehicle_local_position.z < self.takeoff_height + 0.5:
                self.taken_off = True
                print("taken off")

        if self.taken_off and not self.first_setpoint:
            if self.farthest_corner and self.center_point and self.distance:
                # 이 조건을 조금 더 세련되게 바꿔야지만 장애물 회피 조건 맞춰줘야함
                obstacle_radius = self.distance.data  # 장애물의 크기
                margin = 1.0  # 추가적인 마진 거리를 좀 길게 잡아야 될 것 같은데...
                adjusted_distance = obstacle_radius + margin  # 마진을 적용한 장애물의 크기
                target_x, target_y = self.center_point.z, self.center_point.x  # 장애물의 중심점 좌표
                # 카메라 좌표계랑 PX4 좌표계 차이로 인해서, 카메라 좌표계의 z와 x를 가져와야함

                current_x = self.vehicle_local_position.x # 내 현재 위치 x (PX4 기준)
                current_y = self.vehicle_local_position.y # 내 현재 위cl y (PX4 기준)

                distance_to_obstacle = math.sqrt((target_x-current_x) ** 2 + (target_y-current_y) ** 2)
                # 나랑 장애물 사이의 거리

                if (distance_to_obstacle <= adjusted_distance) and (self.vehicle_local_position.x >= 2):
                    vx, vy = self.avoid_from_target(target_x, target_y)
                    self.publish_velocity_setpoint(vx, vy, 0.0)

                else:
                    # 장애물이 없을 때 기본 궤적 유지
                    vx, vy, vz = self.move_towards_target(20.0, 0.0, self.takeoff_height)
                    self.publish_velocity_setpoint(vx, vy, vz)
                    #self.get_logger().info(f"Moving towards target: vx={vx}, vy={vy}, vz={vz}")
                    if self.vehicle_local_position.x > 19:
                        self.first_setpoint = True
                        #self.get_logger().info(f"Location: {self.vehicle_local_position.x}, {self.vehicle_local_position.y}")
            else:
                # 센서 데이터가 없을 때 기본 궤적 유지
                vx, vy, vz = self.move_towards_target(20.0, 0.0, self.takeoff_height)
                self.publish_velocity_setpoint(vx, vy, vz)
                #self.get_logger().info(f"Moving towards target: vx={vx}, vy={vy}, vz={vz}")
                if self.vehicle_local_position.x > 19:
                    self.first_setpoint = True
                    #self.get_logger().info(f"Location: {self.vehicle_local_position.x}, {self.vehicle_local_position.y}")

        if self.vehicle_local_position.x > 19.85 and self.first_setpoint:
            self.get_logger().info(f"Location: {self.vehicle_local_position.x}, {self.vehicle_local_position.y}")
            self.land()
            exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
