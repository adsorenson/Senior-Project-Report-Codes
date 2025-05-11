import geometry_msgs.msg
from std_msgs.msg import Float32
import rclpy
from pynput import keyboard
from dataclasses import dataclass
import serial
import time

# Serial port configuration - adjust the port name as needed
SERIAL_PORT = '/dev/ttyACM0'  # For Linux; use 'COM3' or similar for Windows
BAUD_RATE = 115200

# Update rate in Hz
UPDATE_RATE_HZ = 50

# Messages for instructions
msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages and sends commands via serial to the Arduino.
---------------------------
Moving around:
   w
a  s  d

Space: Toggle brake
Enter: Toggle drive mode
CTRL-C to quit
"""

@dataclass
class Velocity:
    brake: int = 0
    mode: int = 0
    linear: float = 0.0
    angular: float = 0.0

# Define movement and turning
move = {
    'w': (1, 0, 0, 0),  # Move forward
    's': (-1, 0, 0, 0),  # Move backward
}
turn = {
    'a': (0, 0, 0, -1),  # Turn left
    'd': (0, 0, 0, 1),  # Turn right
}

# Set to track current keys pressed (for movement keys and to prevent repeated toggles)
pressed_keys = set()

# Gradual movement constants
max_linear_speed = 1.5  # Max forward/backward speed
linear_increment = 1.5  # Acceleration rate
linear_decrement = 1.5  # Deceleration rate

max_turn_speed = 0.7     # Max turn speed
turn_increment = 0.1     # Turning acceleration
turn_decrement = 0.1     # Turning deceleration rate

# Initialize velocity struct
velocity = Velocity()

# Serial connection
serial_conn = None

def setup_serial():
    """Set up the serial connection to the Arduino."""
    global serial_conn
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  # Faster timeout
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        time.sleep(1)  # Reduced from 2 to 1 second
        return True
    except Exception as e:
        print(f"Failed to connect to serial port: {e}")
        return False

def send_serial_command():
    """Send the current velocity state via serial."""
    global velocity, serial_conn
    
    if serial_conn is None or not serial_conn.is_open:
        return
    
    # Format the command string - shortened for efficiency
    command = f"B:{velocity.brake},M:{velocity.mode},L:{velocity.linear:.2f},A:{velocity.angular:.2f}\n"
    
    try:
        serial_conn.write(command.encode())
        serial_conn.flush()  # Ensure data is sent immediately
    except Exception as e:
        print(f"Error sending serial command: {e}")

def on_press(key):
    """Callback function for key press events."""
    try:
        # For toggle keys, only act if the key is not already recorded as pressed.
        if key == keyboard.Key.space:
            pressed_keys.add(key)
        elif key == keyboard.Key.enter:
            # Only toggle mode if both linear and angular velocities are zero.
            if velocity.linear == 0.0 and velocity.angular == 0.0:
                if key not in pressed_keys:
                    velocity.mode = (velocity.mode + 1) % 4
                    if velocity.mode == 0:
                        print('\nNeutral Mode')
                    elif velocity.mode == 1:
                        print('\nDrive Mode')
                    elif velocity.mode == 2:
                        print('\nSport Mode')
                    else:
                        print('\nReverse Mode')
            pressed_keys.add(key)
        elif hasattr(key, 'char') and key.char in {'w', 'a', 's', 'd'}:
            pressed_keys.add(key.char)  # Movement keys stored as string.
    except AttributeError:
        pass

def on_release(key):
    """Callback function for key release events."""
    try:
        if key == keyboard.Key.space:
            pressed_keys.discard(key)
        elif key == keyboard.Key.enter:
            pressed_keys.discard(key)
        elif hasattr(key, 'char') and key.char in {'w', 'a', 's', 'd'}:
            pressed_keys.discard(key.char)
        if key == keyboard.Key.esc:
            return False  # Stop the listener.
    except KeyError:
        pass

def update():
    """Update the linear and angular velocities based on key presses."""
    global velocity

    if keyboard.Key.space in pressed_keys:
        velocity.brake = 1
    else:
        velocity.brake = 0

    moving = False
    turning = False

    # Process movement keys only; toggle keys are handled in the on_press callback.
    for key in pressed_keys.copy():
        if isinstance(key, str):
            if key in move:
                x, _, _, _ = move[key]
                if x > 0:
                    velocity.linear = min(velocity.linear + linear_increment, max_linear_speed)
                elif x < 0:
                    velocity.linear = max(velocity.linear - linear_increment, -max_linear_speed)
                moving = True
            elif key in turn:
                _, _, _, th = turn[key]
                if th > 0:
                    velocity.angular = min(velocity.angular + turn_increment, max_turn_speed)
                elif th < 0:
                    velocity.angular = max(velocity.angular - turn_increment, -max_turn_speed)
                turning = True

    # Gradual deceleration when no movement keys are pressed.
    if not moving:
        if velocity.linear > 0:
            velocity.linear = max(0, velocity.linear - linear_decrement)
        else:
            velocity.linear = min(0, velocity.linear + linear_decrement)
    if not turning:
        if velocity.angular > 0:
            velocity.angular = max(0, velocity.angular - turn_decrement)
        else:
            velocity.angular = min(0, velocity.angular + turn_decrement)

def main():
    global velocity

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')
    
    # Original publisher
    keyboard_pub = node.create_publisher(geometry_msgs.msg.Twist, '/keyboard/cmd_vel', 10)
    # New publisher for the topic your data logger is listening to
    twist_mux_pub = node.create_publisher(geometry_msgs.msg.Twist, '/twist_mux/cmd_vel', 10)
    # New publishers for speed and brake data
    speed_pub = node.create_publisher(Float32, '/vehicle/speed', 10)
    brake_pub = node.create_publisher(Float32, '/vehicle/brake', 10)
    
    if not setup_serial():
        print("Warning: Serial connection not established. Continuing without Arduino control.")
    
    try:
        print(msg)
        
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        
        # Calculate sleep time based on update rate
        update_interval = 1.0 / UPDATE_RATE_HZ
        last_display_time = time.time()
        display_interval = 0.1  # Display status every 100ms instead of every cycle
        
        while rclpy.ok():
            update_start = time.time()
            update()
            
            # Create Twist message
            twist = geometry_msgs.msg.Twist()
            # If brake is active, override movement by zeroing out the linear velocity.
            if velocity.brake:
                twist.linear.x = 0.0
            else:
                twist.linear.x = float(velocity.linear)  # Explicitly convert to float
            twist.angular.z = float(velocity.angular)    # Explicitly convert to float
            
            # Publish to original ROS topic
            keyboard_pub.publish(twist)
            # Also publish to the twist_mux topic (for data logger)
            twist_mux_pub.publish(twist)
            
            # Create and publish speed message
            speed_msg = Float32()
            # Use absolute value for speed since it's magnitude
            speed_msg.data = float(abs(velocity.linear))  # Explicitly convert to float
            speed_pub.publish(speed_msg)
            
            # Create and publish brake message
            brake_msg = Float32()
            brake_msg.data = float(velocity.brake)  # Convert int to float
            brake_pub.publish(brake_msg)
            
            # Send command to Arduino every cycle
            send_serial_command()
            
            # Display status less frequently to avoid console spam
            current_time = time.time()
            if current_time - last_display_time >= display_interval:
                print(f'Brake: {velocity.brake} Mode: {velocity.mode} Direction: {twist.linear.x:.2f} Turn: {twist.angular.z:.2f}')
                last_display_time = current_time
            
            # Calculate remaining time to maintain update rate
            elapsed = time.time() - update_start
            sleep_time = max(0, update_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Only spin ROS when needed
            rclpy.spin_once(node, timeout_sec=0)
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Cleanup
        listener.stop()
        
        # Send stop commands to ROS topics
        twist = geometry_msgs.msg.Twist()
        keyboard_pub.publish(twist)
        twist_mux_pub.publish(twist)  # Also send stop command to twist_mux
        
        # Send zero values to speed and brake
        zero_msg = Float32()
        zero_msg.data = 0.0
        speed_pub.publish(zero_msg)
        brake_pub.publish(zero_msg)
        
        # Close serial connection
        if serial_conn and serial_conn.is_open:
            # Send a stop command
            stop_cmd = "B:0,M:0,L:0.00,A:0.00\n"
            serial_conn.write(stop_cmd.encode())
            serial_conn.close()
            print("Serial connection closed")

if __name__ == '__main__':
    main()
