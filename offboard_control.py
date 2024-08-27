#!/usr/bin/env python3

"뭔 이딴 코드가 생겼지 근데 왜 될까.. P턴 추가해야되고, 잘하면 ros2 custom message 파가지고 Mavlink 연결해서 미션 업로드 하겠습니다"
"ROS2 DDS 찾아보니까 아직 Mission_ITEMS해가지고 미션 업로드 하는게 구현이 안된거 같음"

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleGlobalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
from pymavlink import mavutil

class DroneMissionController(Node):

    def __init__(self):
        super().__init__('drone_mission_controller')

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscribers
        self.global_position_subscriber = self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # MAVLink connection
        try:
            self.master = mavutil.mavlink_connection('udpin:localhost:14540', timeout=10)
            self.get_logger().info("MAVLink connection established")
        except Exception as e:
            self.get_logger().error(f"Failed to establish MAVLink connection: {e}")
            raise

        self.current_position = None
        self.vehicle_status = None
        self.flight_stage = "INIT"
        self.takeoff_altitude = 10.0  # Takeoff altitude in meters
        self.current_waypoint = 0

        # Waypoints
        self.waypoints = [
            {"lat": 36.661078, "lon": 126.342193, "alt": 50.0, "action": "transition_fw"},
            {"lat": 36.662081, "lon": 126.344977, "alt": 50.0, "action": None},
            {"lat": 36.663238, "lon": 126.341070, "alt": 50.0, "action": None},
            {"lat": 36.661950, "lon": 126.341837, "alt": 10.0, "action": None},
            {"lat": 36.660259, "lon": 126.342790, "alt": 10.0, "action": None},
            {"lat": 36.659412, "lon": 126.344157, "alt": 30.0, "action": "transition_mc"},
            {"lat": 36.660226, "lon": 126.342404, "alt": 30.0, "action": None},
            {"lat": 36.661078, "lon": 126.342193, "alt": 5.0, "action": None},
        ]

        # Timer for periodic checks
        self.create_timer(1.0, self.mission_callback)

    def global_position_callback(self, msg):
        self.current_position = {"lat": msg.lat, "lon": msg.lon, "alt": msg.alt}
        self.get_logger().debug(f"Current position: Lat: {msg.lat}, Lon: {msg.lon}, Alt: {msg.alt}m")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.get_logger().debug(f"Vehicle status - Armed: {msg.arming_state == VehicleStatus.ARMING_STATE_ARMED}, Nav state: {msg.nav_state}")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Arm command sent")

    def takeoff(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=float('nan'),  # Yaw
            param5=float('nan'),  # Latitude
            param6=float('nan'),  # Longitude
            param7=self.takeoff_altitude
        )
        self.get_logger().info(f"Takeoff command sent, target altitude: {self.takeoff_altitude}m")

    def transition_fw(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=4.0)
        self.get_logger().info("Transition to fixed-wing command sent")

    def transition_mc(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=3.0)
        self.get_logger().info("Transition to multi-copter command sent")

    def wait_heartbeat(self):
        self.get_logger().info("Waiting for heartbeat...")
        try:
            self.master.wait_heartbeat(timeout=10)
            self.get_logger().info("Heartbeat received")
            return True
        except mavutil.mavlink.MAVError:
            self.get_logger().error("No heartbeat received")
            return False

    def upload_mission(self):
        self.get_logger().info("Starting mission upload...")

        if not self.wait_heartbeat():
            self.get_logger().error("Failed to receive heartbeat, aborting mission upload")
            return False

        try:
            self.get_logger().info("Clearing existing mission...")
            self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
            self.get_logger().info("Waiting for mission clear acknowledgement...")
            ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if not ack:
                self.get_logger().error("No acknowledgement received for mission clear")
                return False
            self.get_logger().info("Existing mission cleared")

            mission_items = []
            for i, wp in enumerate(self.waypoints):
                mission_items.append(
                    self.master.mav.mission_item_int_encode(
                        self.master.target_system,
                        self.master.target_component,
                        i,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1,
                        0, 0, 0, float('nan'),
                        int(wp['lat'] * 1e7), int(wp['lon'] * 1e7), wp['alt']
                    )
                )

            self.get_logger().info(f"Sending mission item count: {len(mission_items)}")
            self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, len(mission_items))

            for i in range(len(mission_items)):
                self.get_logger().info(f"Waiting for mission request {i}...")
                msg = self.master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=10)
                if msg is None:
                    self.get_logger().error(f"No mission request received for item {i}")
                    return False
                self.get_logger().info(f"Sending mission item {msg.seq}")
                self.master.mav.send(mission_items[msg.seq])

            self.get_logger().info("Waiting for mission acknowledgement...")
            msg = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if msg is None:
                self.get_logger().error("No mission acknowledgement received")
                return False
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                self.get_logger().info("Mission uploaded successfully")
                return True
            else:
                self.get_logger().error(f"Mission upload failed: {msg.type}")
                return False

        except Exception as e:
            self.get_logger().error(f"Exception during mission upload: {e}")
            return False

    def start_mission(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Mission start command sent")

    def mission_callback(self):
        if self.current_position is None or self.vehicle_status is None:
            self.get_logger().info("Waiting for vehicle data...")
            return

        self.get_logger().info(f"Current flight stage: {self.flight_stage}")

        if self.flight_stage == "INIT":
            if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
            else:
                self.flight_stage = "UPLOAD_MISSION"

        elif self.flight_stage == "UPLOAD_MISSION":
            if self.upload_mission():
                self.flight_stage = "TAKEOFF"
            else:
                self.get_logger().error("Mission upload failed, retrying in next iteration")

        elif self.flight_stage == "TAKEOFF":
            self.takeoff()
            self.flight_stage = "WAIT_FOR_TAKEOFF"

        elif self.flight_stage == "WAIT_FOR_TAKEOFF":
            if self.current_position['alt'] >= self.takeoff_altitude - 1.0:  # 1m tolerance
                self.flight_stage = "START_MISSION"

        elif self.flight_stage == "START_MISSION":
            self.start_mission()
            self.flight_stage = "MONITOR_MISSION"

        elif self.flight_stage == "MONITOR_MISSION":
            msg = self.master.recv_match(type='MISSION_ITEM_REACHED', blocking=False)
            if msg:
                self.get_logger().info(f"Reached waypoint {msg.seq}")
                if msg.seq < len(self.waypoints) - 1:
                    current_wp = self.waypoints[msg.seq]
                    if current_wp["action"] == "transition_fw":
                        self.transition_fw()
                    elif current_wp["action"] == "transition_mc":
                        self.transition_mc()
                else:
                    self.get_logger().info("Mission completed")
                    self.flight_stage = "MISSION_COMPLETE"

        elif self.flight_stage == "MISSION_COMPLETE":
            self.land()
            self.get_logger().info("Mission completed. Holding position.")

def main(args=None):
    rclpy.init(args=args)
    node = DroneMissionController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
