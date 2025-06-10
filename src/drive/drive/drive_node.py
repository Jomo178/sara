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
        
        self.get_logger().info(f"Drive node initialized with wheelbase: {self.wheelbase}m, max_steering: {self.max_steering_angle}rad")

    def convert_trans_rot_vel_to_steering_angle(self, v, omega):
        """Convert linear and angular velocity to steering angle using Ackermann model"""
        if abs(omega) < 0.001 or abs(v) < 0.001:
            return 0.0
        
        # Calculate turning radius
        radius = v / omega
        
        # Calculate steering angle using Ackermann geometry
        steering_angle = math.atan(self.wheelbase / radius)
        
        # Clamp to maximum steering angle
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        return steering_angle

    def listener_callback(self, msg: Twist):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Motor control - map linear velocity to motor values
        # Your car starts driving at 106, limiting to 108 for testing
        if abs(linear_vel) > 0.01:
            if linear_vel > 0:
                # Forward motion: 106-108 range (limited for testing)
                motor_range = 2  # 108-106
                motor_offset = (linear_vel / self.max_speed) * motor_range
                motor_offset = max(0, min(motor_range, motor_offset))  # Clamp to positive
                self.target_vals[0] = int(106 + motor_offset)
            else:
                # Reverse motion: 88-86 range (symmetric around 90, limited for testing)
                motor_range = 2  # 88-86
                motor_offset = (-linear_vel / self.max_speed) * motor_range
                motor_offset = max(0, min(motor_range, motor_offset))  # Clamp to positive
                self.target_vals[0] = int(88 - motor_offset)
        else:
            self.target_vals[0] = 90  # Stop

        # Steering control using proper Ackermann conversion
        if abs(linear_vel) > 0.01 or abs(angular_vel) > 0.01:
            if abs(angular_vel) > 0.01 and abs(linear_vel) > 0.01:
                # Normal case: both linear and angular velocity provided
                steering_angle = self.convert_trans_rot_vel_to_steering_angle(linear_vel, angular_vel)
            elif abs(angular_vel) > 0.01:
                # Pure rotation - use maximum steering angle
                steering_angle = self.max_steering_angle if angular_vel > 0 else -self.max_steering_angle
            else:
                # Pure linear motion - straight ahead
                steering_angle = 0.0
            
            # Convert steering angle to servo value (60-120, with 90 as center)
            # Negative sign to match your car's steering direction
            angle_ratio = -steering_angle / self.max_steering_angle
            servo_val = int(90 + (angle_ratio * 30))
            servo_val = max(60, min(120, servo_val))
            self.target_vals[1] = servo_val
        else:
            # No movement - center steering
            self.target_vals[1] = 90

        # Use info level logging so you can see the data being sent
        self.get_logger().info(f"Cmd: v={linear_vel:.3f}, ω={angular_vel:.3f} -> motor={self.target_vals[0]}, servo={self.target_vals[1]}")

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
                # Use info level logging to see actual data being sent to car
                self.get_logger().info(f"Sent to car: motor={self.current_vals[0]}, servo={self.current_vals[1]}")
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