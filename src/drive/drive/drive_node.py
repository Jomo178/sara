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
        self.declare_parameter('max_steering_angle', 0.5)  # Max steering angle in radians (about 28 degrees)
        
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial connected on /dev/ttyUSB2")
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
        
        self.get_logger().info(f"Drive node initialized with wheelbase: {self.wheelbase}m, max_steering: {self.max_steering_angle}rad")

    def convert_trans_rot_vel_to_steering_angle(self, v, omega):
        """Convert linear and angular velocity to steering angle using Ackermann model"""
        if abs(omega) < 0.001:
            return 0.0
 
        # Invert omega to match ROS2 convention: positive omega = left turn
        steering_angle = math.atan((self.wheelbase * -omega))

        return steering_angle

    def listener_callback(self, msg: Twist):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        steering_angle = 0.0  # Initialize steering angle

        # Motor control - map linear velocity to motor values
        if abs(linear_vel) > 0.01:
            if linear_vel > 0:
                # Forward motion: 106-108 range (limited for testing)
                motor_range = 2  # 108-106
                motor_offset = (linear_vel / self.max_speed) * motor_range
                motor_offset = max(0, min(motor_range, motor_offset))  # Clamp to positive
                self.target_vals[0] = int(110 + motor_offset)
            else:
                # Reverse motion: 88-86 range (symmetric around 90, limited for testing)
                motor_range = 1  # 88-86
                motor_offset = (-linear_vel / self.max_speed) * motor_range
                motor_offset = max(0, min(motor_range, motor_offset))  # Clamp to positive
                self.target_vals[0] = int(75 - motor_offset)
        else:
            self.target_vals[0] = 90  # Stop

        # Steering control using Ackermann conversion
        if abs(linear_vel) > 0.01 or abs(angular_vel) > 0.01:
            # Always use Ackermann model for consistent steering behavior
            steering_angle = self.convert_trans_rot_vel_to_steering_angle(linear_vel, angular_vel)
            
            # Convert steering angle to servo value (60-120, with 90 as center)
            # Map steering angle to servo range: -max_angle -> 60, 0 -> 90, +max_angle -> 120
            angle_ratio = steering_angle / self.max_steering_angle
            # Clamp the ratio to [-1, 1] to ensure servo stays in valid range
            angle_ratio = max(-1.0, min(1.0, angle_ratio))
            servo_val = int(90 + (angle_ratio * 30))
            servo_val = max(60, min(120, servo_val))
            self.target_vals[1] = servo_val
        else:
            # No movement - center steering
            self.target_vals[1] = 90

        # Use info level logging so you can see the data being sent
        self.get_logger().info(f"Cmd: v={linear_vel:.3f}, ω={angular_vel:.3f} -> motor={self.target_vals[0]}, servo={self.target_vals[1]} (steering_angle={steering_angle:.3f}rad)")
        try:
            self.ser.write(bytearray(self.target_vals))
            self.get_logger().info(f"Sent to car: motor={self.target_vals[0]}, servo={self.target_vals[1]}")
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