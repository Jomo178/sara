import serial
import time
import keyboard

#ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)    
#ser.reset_input_buffer()

motor_servo_vals = [511, 511] 
key_states = {'w': False, 's': False, 'a': False, 'd': False}

def update_values():
    global motor_servo_vals 
    if key_states['w']:
        motor_servo_vals[0] = 600
    elif key_states['s']:
        motor_servo_vals[0] = 600
    else:
        motor_servo_vals[0] = 511
 
    if key_states['a']:
        motor_servo_vals[1] = 400
    elif key_states['d']:
        motor_servo_vals[1] = 600
    else:
        motor_servo_vals[1] = 511
    
    #print(motor_servo_vals)
    #ser.write(bytearray(motor_servo_vals))
    #line = ser.readline().decode('latin-1').rstrip()

def on_key_event(event):
    global key_states
    if event.name in key_states:
        print(key_states)
        if event.event_type == keyboard.KEY_DOWN:
            key_states[event.name] = True
        elif event.event_type == keyboard.KEY_UP:
            key_states[event.name] = False
        update_values()

keyboard.hook(on_key_event)

print("Press 'wsad' keys to track. Press 'esc' to exit.")
keyboard.wait('esc')