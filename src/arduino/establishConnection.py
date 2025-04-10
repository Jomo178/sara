import serial
import time
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    while True:
        motor_servo_vals = [90, 80] #MOTOR, SERVO
        ser.write(bytearray(motor_servo_vals))
        line = ser.readline().decode('latin-1').rstrip() #READ DATA
        print(line)
        time.sleep(1)

# ser.write(str(VALUE).encode('utf-8')) to send Datas