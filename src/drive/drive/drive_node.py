import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class SerialDriveNode(Node):
    def __init__(self):
        super().__init__('serial_drive_node')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.31)  # Default wheelbase in meters
        self.declare_parameter('max_speed', 2.0)  # Max speed in m/s
        self.declare_parameter('max_steering_angle', 0.01)  # Max steering angle in radians
        
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

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        
        self.get_logger().info(f"Drive node initialized with wheelbase: {self.wheelbase}m, max_steering: {self.max_steering_angle}rad")

    def convert_trans_rot_vel_to_steering_angle(self, v, omega):
        """Convert linear and angular velocity to steering angle using Ackermann model"""
        if abs(omega) < 0.001 or abs(v) < 0.001:
            return 0.0
        
        # Calculate turning radius
        radius = v / omega
        
        # Calculate steering angle using Ackermann geometry
        steering_angle = math.atan((self.wheelbase / (radius - (0.17/2))))
        
        # Clamp to maximum steering angle
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        return steering_angle

    def listener_callback(self, msg: Twist):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Initialize motor and servo values
        motor_val = 90  # Stop by default
        servo_val = 90  # Center steering by default

        # Motor control - map linear velocity to motor values
        if abs(linear_vel) > 0.01:
            if linear_vel > 0:
                # Forward motion: 106-120 range
                motor_range = 14  # 120-106
                motor_offset = (linear_vel / self.max_speed) * motor_range
                motor_offset = max(0, min(motor_range, motor_offset))  # Clamp to positive
                motor_val = int(106 + motor_offset)
            else:
                # Reverse motion: 80-60 range
                motor_range = 20  # 80-60
                motor_offset = (-linear_vel / self.max_speed) * motor_range
                motor_offset = max(0, min(motor_range, motor_offset))  # Clamp to positive
                motor_val = int(80 - motor_offset)

        # Steering control using proper Ackermann conversion
        if abs(angular_vel) > 0.01:
            if abs(linear_vel) > 0.01:
                # Normal case: both linear and angular velocity provided
                steering_angle = self.convert_trans_rot_vel_to_steering_angle(linear_vel, angular_vel)
            else:
                # Pure rotation - use maximum steering angle
                steering_angle = self.max_steering_angle if angular_vel > 0 else -self.max_steering_angle
            
            # Convert steering angle to servo value (60-120, with 90 as center)
            angle_ratio = steering_angle / self.max_steering_angle
            servo_val = int(90 + (angle_ratio * 30))
            servo_val = max(60, min(120, servo_val))
        else:
            # No angular velocity - straight ahead
            servo_val = 90

        # Send values directly to car - no gradual transition
        try:
            self.ser.write(bytearray([motor_val, servo_val]))
            self.get_logger().info(f"Cmd: v={linear_vel:.3f}, ω={angular_vel:.3f} -> motor={motor_val}, servo={servo_val}")
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