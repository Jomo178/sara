import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class SerialDriveNode(Node):
    def __init__(self):
        super().__init__('serial_drive_node')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.26)  # Default wheelbase in meters
        self.declare_parameter('max_speed', 1.0)  # Max speed in m/s
        self.declare_parameter('max_steering_angle', 0.5)  # Max steering angle in radians
        
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial connected on /dev/ttyUSB0")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            return

        self.current_vals = [90, 90]  # [motor, servo]
        self.target_vals = [90, 90]
        self.step_size = 1

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.update_output)
        
        self.get_logger().info(f"Drive node initialized with wheelbase: {self.wheelbase}m")

    def convert_trans_rot_vel_to_steering_angle(self, v, omega):
        """Convert linear and angular velocity to steering angle using Ackermann model"""
        if omega == 0 or v == 0:
            return 0
        
        radius = v / omega
        return math.atan(self.wheelbase / radius)

    def listener_callback(self, msg: Twist):
        throttle = msg.linear.x
        angular_vel = msg.angular.z

        # Drive at constant speed (108) when there's any movement command
        if abs(throttle) > 0.01 or abs(angular_vel) > 0.01:
            self.target_vals[0] = 108  # Constant speed
        else:
            self.target_vals[0] = 90  # Stop when no command

        # Handle steering
        if abs(angular_vel) > 0.01 and abs(throttle) > 0.01:
            # Standard case: both speed and angular velocity provided
            steering_angle = self.convert_trans_rot_vel_to_steering_angle(throttle, angular_vel)
            # Clamp steering angle to maximum
            steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
            # Convert steering angle to servo value (60-120, with 90 as center)
            angle_ratio = steering_angle / self.max_steering_angle
            servo_val = int(90 - (angle_ratio * 30))  # Note: negative for correct direction
            servo_val = max(60, min(120, servo_val))
            self.target_vals[1] = servo_val
        elif abs(throttle) > 0.4:  # Speed threshold for turning
            # High speed values indicate turning commands
            if throttle > 0.8:  # Turn right when speed > 0.8
                servo_val = 60  # Full right turn
                self.target_vals[1] = servo_val
                self.get_logger().info(f"Right turn: speed={throttle:.2f}")
            elif throttle < -0.8:  # Turn left when speed < -0.8
                servo_val = 120  # Full left turn
                self.target_vals[1] = servo_val
                self.get_logger().info(f"Left turn: speed={throttle:.2f}")
            elif throttle > 0.4:  # Moderate right turn
                servo_val = 75  # Moderate right
                self.target_vals[1] = servo_val
                self.get_logger().info(f"Moderate right: speed={throttle:.2f}")
            elif throttle < -0.4:  # Moderate left turn
                servo_val = 105  # Moderate left
                self.target_vals[1] = servo_val
                self.get_logger().info(f"Moderate left: speed={throttle:.2f}")
        elif abs(throttle) > 0.01 and abs(angular_vel) <= 0.01:
            # Low speed: straight driving
            self.target_vals[1] = 90  # Center steering for straight driving
            self.get_logger().debug(f"Straight driving: speed={throttle:.2f}")
        else:
            # No movement
            self.target_vals[1] = 90  # Center steering

        self.get_logger().debug(f"Cmd: speed={throttle:.2f}, angular={angular_vel:.2f} -> motor={self.target_vals[0]}, servo={self.target_vals[1]}")

    def update_output(self):
        updated = False

        for i in range(2):
            if self.current_vals[i] < self.target_vals[i]:
                self.current_vals[i] = min(self.current_vals[i] + self.step_size, self.target_vals[i])
                updated = True
            elif self.current_vals[i] > self.target_vals[i]:
                self.current_vals[i] = max(self.current_vals[i] - self.step_size, self.target_vals[i])
                updated = True

        if updated:
            try:
                self.ser.write(bytearray(self.current_vals))
                self.get_logger().info(f"Sent: {self.current_vals}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser'):
            node.ser.write(bytearray([90, 90]))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#ros2 run teleop_twist_keyboard teleop_twist_keyboard