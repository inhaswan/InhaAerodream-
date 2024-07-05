import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray, String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import numpy as np

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.vector_subscriber = self.create_subscription(
            Float32MultiArray, '/vector', self.vector_callback, qos_profile)
        self.vector_subscriber = self.create_subscription(
            String, '/disarm', self.disarm, qos_profile)

        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vector_subscriber
        self.takeoff_height = -2.0
        self.vector_x = 0.0
        self.vector_y = 0.0
        self.tolerance = 0.1
        self.new_vector_subscribed = False
        self.landing_flag = False        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def vector_callback(self, msg):
        self.vector_x = msg.data[0]
        self.vector_y = msg.data[1]
        self.new_vector_subscribed = True

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_velocity_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'),float('nan'),float('nan')]
        msg.velocity = [x, y, z]
        msg.acceleration = [float('nan'),float('nan'),float('nan')]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
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

    def timer_callback(self) -> None:
        self.offboard_setpoint_counter += 1
        self.publish_offboard_control_heartbeat_signal()
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        
        if self.offboard_setpoint_counter == 50:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        
        if abs(self.vehicle_local_position.z - self.takeoff_height) <= self.tolerance:
             self.publish_position_setpoint(1.0, 1.0, self.takeoff_height)

        # arrived land point -> landing flag ON
        if (abs(self.vehicle_local_position.x - 1) <= self.tolerance and
            abs(self.vehicle_local_position.y - 1) <= self.tolerance and
            abs(self.vehicle_local_position.z + 2) <= self.tolerance):
            self.landing_flag = True
            
        # subscrined vector -> publish vel_cmd
        if self.landing_flag == True and self.new_vector_subscribed:
            self.vector = np.sqrt(self.vector_x**2 + self.vector_y**2)
            if self.vector < 100:
                self.publish_velocity_setpoint(self.vector_x/100, self.vector_y/100, 0.1)
            else:
                self.publish_velocity_setpoint(self.vector_x/100, self.vector_y/100, 0.0)
            self.new_vector_subscribed = False
        
        #less than 0.2m -> land (but there is another disarm command)
        # if self.vehicle_local_position.z <= -0.2:
        #     self.disarm()
        #     exit(0)



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
