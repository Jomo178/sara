import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.reset_input_buffer()

motor_servo_vals = [90, 90] 
last_sent_vals = [90, 90]  # Track the last values sent

def getch():
    import sys, termios, tty

    fd = sys.stdin.fileno()
    orig = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)  
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, orig)


#servo max: 120 d; 60 a;
def movementIsValid(value, operation):

    if value + operation >= 60 and value + operation <= 120:
        return True
    else:
        return False

def update_values(key):
    global motor_servo_vals, last_sent_vals
    
    if key == 'w':
        if movementIsValid(motor_servo_vals[0],2):
            motor_servo_vals[0] = motor_servo_vals[0] + 2
    elif key == 's':
        if movementIsValid(motor_servo_vals[0], -2):
            motor_servo_vals[0] = motor_servo_vals[0] - 2
    elif key == 'q':
        motor_servo_vals[0] = 90
    else:
       motor_servo_vals[0] = 90
        
    if key == 'a':
        if movementIsValid(motor_servo_vals[1], -10):
            motor_servo_vals[1] = motor_servo_vals[1] - 10
    elif key == 'd':
        if movementIsValid(motor_servo_vals[1], 10):
            motor_servo_vals[1] = motor_servo_vals[1] + 10
    elif key == 'e':
        motor_servo_vals[1] = 90
    else:
       motor_servo_vals[1] = 90
    
    # Only send if values have changed
    if motor_servo_vals != last_sent_vals:
        ser.write(bytearray(motor_servo_vals))
        print("Sent:", motor_servo_vals)
        last_sent_vals = motor_servo_vals.copy() 

print("Press 'w' (forward), 's' (backward), 'a' (left), 'd' (right), 'q' (reset throttle), e' (reset steer)")
print("Press 'f' to exit.")

try:
    while True:
        key = getch()
        #print()
        if key.lower() == 'f':
            break
        if key.lower() in ['w', 's', 'a', 'd', 'e', 'q']:
            update_values(key.lower())
        else:
            print("Invalid command. Use w/s/a/d to control, q to quit.")
            
except KeyboardInterrupt:
    pass
finally:
    # Reset to neutral before exiting
    ser.write(bytearray([90, 90]))
    print("Exiting and resetting to neutral position")

