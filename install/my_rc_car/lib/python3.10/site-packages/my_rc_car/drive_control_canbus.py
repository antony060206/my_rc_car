import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import can, os
import struct
import math


class rc_can_drive(Node):
    def __init__(self):
        super().__init__('rc_can_drive')  # <-- fixed name

        # Subscriptions
        from rclpy.qos import qos_profile_sensor_data #test
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.rotate_sub = self.create_subscription(Joy, '/manual/joy', self.rotate_rover, 10)

        # Defaults so we don't use uninitialized values
        self.v = 0.0
        self.omega = 0.0
        self.rotate = 0

        # Rover geometry (SET THESE to your real dimensions, meters)
        self.L1 = 0.40   # front axle to ICC lever arm
        self.L2 = 0.40   # rear axle to ICC lever arm
        self.T  = 0.50   # track width

        # CAN bus
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1_000_000)


        #test
        
        # self.bus = None
        # self.bustype = os.getenv("CAN_BUSTYPE", "socketcan")
        # self.channel = os.getenv("CAN_CHANNEL", "can0")
        # self._try_open_bus()
        

        # Timer
        self.timer = self.create_timer(0.25, self.paced_commands)

        self.get_logger().info("CAN drive node started")

    def rotate_rover(self, msg: Joy):
        # Use LB on Xbox as example; guard array access
        if len(msg.buttons) > 4:
            self.rotate = msg.buttons[4]
        else:
            self.rotate = 0

    def cmd_vel_callback(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.omega = float(msg.angular.z)

    def paced_commands(self):
        try:
            if self.rotate != 1:
                steering_angles = self.steering_angle(self.v, self.omega)
                wheel_velocities = self.wheel_velocities(self.v, self.omega)

                # Send steering
                for actuator_id, angle in steering_angles.items():
                    can_id = self.steering_actuator_id_to_can(actuator_id)
                    if can_id is None:
                        continue
                    message = self.create_steering_command(can_id, angle)
                    self.bus.send(message)

                # Send drive
                for actuator_id, velocity in wheel_velocities.items():
                    can_id = self.drive_actuator_id_to_can(actuator_id)
                    if can_id is None:
                        continue
                    message = self.create_drive_command(can_id, velocity)
                    self.bus.send(message)

            else:
                rotate_angles = self.calculate_rotate_angle()
                wheel_velocities = self.wheel_velocities(self.v, self.omega)  # fixed name

                for actuator_id, angle in rotate_angles.items():
                    can_id = self.steering_actuator_id_to_can(actuator_id)
                    if can_id is None:
                        continue
                    message = self.create_steering_command(can_id, angle)
                    self.bus.send(message)

                for actuator_id, velocity in wheel_velocities.items():
                    can_id = self.drive_actuator_id_to_can(actuator_id)
                    if can_id is None:
                        continue
                    message = self.create_drive_command(can_id, velocity)
                    self.bus.send(message)

        except can.CanError as e:
            self.get_logger().error(f"CAN send failed: {e}")

    def calculate_rotate_angle(self):
        # radians; adjust as needed
        return {"front_left": -0.880, "front_right": 0.880, "back_left": 1.035, "back_right": -1.035}

    # ---- ID mappings ----
    def steering_actuator_id_to_can(self, actuator_id: str):
        mapping = {
            "front_left": 0x25,   # check your wiring vs comments
            "front_right": 0x23,
            "back_left": 0x24,
            "back_right": 0x22,
        }
        return mapping.get(actuator_id)

    def drive_actuator_id_to_can(self, actuator_id: str):
        mapping = {
            "fl": 0x14,
            "fr": 0x12,
            "ml": 0x21,
            "mr": 0x26,
            "bl": 0x13,
            "br": 0x11,
        }
        return mapping.get(actuator_id)

    # ---- Kinematics ----
    def wheel_velocities(self, v: float, omega: float):
        velocity_limiter = 0.5
        v = max(-velocity_limiter, min(velocity_limiter, v))

        # Simple differential for left/right sides; sign for right side may depend on your motor wiring
        vl = v - 0.0 * omega  # placeholder if you later incorporate skid/ICC model
        vr = v + 0.0 * omega

        return {"fl": vl, "fr": -vr, "ml": vl, "mr": -vr, "bl": vl, "br": -vr}

    def steering_angle(self, v: float, omega: float):
        if abs(omega) < 1e-6:
            return {"front_left": 0.0, "front_right": 0.0, "back_left": 0.0, "back_right": 0.0}

        # If v is ~0 but you still have omega, set a nominal forward component so r=v/omega is finite
        v_eff = v if abs(v) > 1e-3 else 1.0 * (1 if omega >= 0 else -1)
        r = v_eff / omega

        # Ackermann-ish angles for a split wheelbase front/back
        fl = math.atan(self.L1 / (r - self.T / 2.0))
        fr = math.atan(self.L1 / (r + self.T / 2.0))
        bl = -math.atan(self.L2 / (r - self.T / 2.0))
        br = -math.atan(self.L2 / (r + self.T / 2.0))
        return {"front_left": fl, "front_right": fr, "back_left": bl, "back_right": br}

    # ---- CAN messages ----
    def create_drive_command(self, actuator_id: int, velocity: float = 0.0):
        priority = 0x0
        command_id = 0x03
        receiver_node_id = actuator_id
        sender_node_id = 1
        arbitration_id = (priority << 24) | (command_id << 16) | (receiver_node_id << 8) | sender_node_id
        data = struct.pack(">f", velocity)
        return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)

    
    def create_steering_command(self, actuator_id: int, angle: float = 0.0):
        priority = 0x0
        command_id = 0x02
        receiver_node_id = actuator_id
        sender_node_id = 1
        arbitration_id = (priority << 24) | (command_id << 16) | (receiver_node_id << 8) | sender_node_id

        # TODO: confirm your controller expects radians, degrees, or a scale. You had "* -50".
        data = struct.pack(">f", angle * -50.0)

        return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
    #test
    def _try_open_bus(self):
        if self.bus is not None:
            return
        try:
            self.bus = can.interface.Bus(bustype=self.bustype, channel=self.channel)
            self.get_logger().info(f"CAN connected via {self.bustype}:{self.channel}")
        except OSError:
            # fallback to in-process virtual bus
            if self.bustype != "virtual":
                try:
                    self.bus = can.interface.Bus(bustype="virtual", channel="sim")
                    self.get_logger().warn("SocketCAN not available; using virtual:sim")
                except OSError:
                    self.get_logger().warn("No CAN backend available yet")
                    self.bus = None
    #test
    def paced_commands(self):
        if self.bus is None:
            self._try_open_bus()
            return




def main(args=None):
    rclpy.init(args=args)
    node = rc_can_drive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
