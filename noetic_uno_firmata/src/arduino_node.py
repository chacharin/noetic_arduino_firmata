#!/usr/bin/env python3

import rospy  # ROS Python library
from std_msgs.msg import String, Float32  # ROS message types
import pyfirmata  # Library for Arduino communication
import threading  # For running sensor reading in a separate thread

# Global variable for the Arduino board
board = None
shutdown_flag = False  # Flag to handle program shutdown

def pin_mode_callback(msg):
    """Set the pin mode on the Arduino based on incoming ROS messages."""
    try:
        pin, mode = msg.data.split(',')  # Expecting a message format like "13,output"
        pin = pin.strip()  # Remove any extra spaces
        mode = mode.strip().lower()  # Convert mode to lowercase for consistency

        # Handle analog pins
        if 'A' in pin:
            pin_number = int(pin[1:])  # Extract the analog pin number
            if mode == "input":
                board.analog[pin_number].enable_reporting()  # Enable input reporting for analog pins
                rospy.loginfo(f"Set Analog Pin A{pin_number} to Input mode")
            elif mode == "disable":
                board.analog[pin_number].disable_reporting()  # Disable reporting for analog pins
                rospy.loginfo(f"Disabled reporting for Analog Pin A{pin_number}")

        # Handle digital pins
        else:
            pin_number = int(pin)  # Convert pin to integer
            if pin_number in [0, 1]:
                rospy.logwarn(f"Cannot set pin {pin_number} (reserved for serial communication)")
                return
            if mode == "input":
                board.digital[pin_number].mode = pyfirmata.INPUT  # Set to input mode
                rospy.loginfo(f"Set Digital Pin {pin_number} to Input mode")
            elif mode == "output":
                board.digital[pin_number].mode = pyfirmata.OUTPUT  # Set to output mode
                rospy.loginfo(f"Set Digital Pin {pin_number} to Output mode")
            elif mode == "pwm":
                board.digital[pin_number].mode = pyfirmata.PWM  # Set to PWM mode
                rospy.loginfo(f"Set Digital Pin {pin_number} to PWM mode")
            elif mode == "servo":
                board.servo_config(pin_number)  # Configure pin for servo control
                rospy.loginfo(f"Configured pin {pin_number} as servo")
    except ValueError:
        rospy.logerr("Invalid input format. Ensure the value is correct.")

def digital_write_callback(msg):
    """Write a digital value to the Arduino pin."""
    try:
        pin, value = msg.data.split(',')  # Expecting a message format like "13,1"
        pin = int(pin)
        if pin in [0, 1]:
            rospy.logwarn(f"Cannot write to pin {pin} (reserved for serial communication)")
            return
        value = int(value)  # Convert value to integer (0 or 1)
        board.digital[pin].write(value)  # Write the digital value to the pin
        rospy.loginfo(f"Wrote digital value {value} to pin {pin}")
    except ValueError:
        rospy.logerr("Invalid input format. Ensure the value is a valid integer.")

def analog_write_callback(msg):
    """Write a PWM value (analog write) to the Arduino pin."""
    try:
        pin, value = msg.data.split(',')
        pin = int(pin)
        if pin in [0, 1]:
            rospy.logwarn(f"Cannot write to pin {pin} (reserved for serial communication)")
            return
        value = float(value)  # Convert the value to a floating point (PWM expects 0.0 to 1.0)
        if 0.0 <= value <= 1.0:
            board.digital[pin].write(value)  # Write the PWM value to the pin
            rospy.loginfo(f"Wrote PWM value {value} to pin {pin}")
        else:
            rospy.logwarn(f"Invalid PWM value {value}. Must be between 0.0 and 1.0.")
    except ValueError:
        rospy.logerr("Invalid input format. Ensure the value is a valid float.")

def servo_write_callback(msg):
    """Write a value to a servo pin (handle servo angles between 0 and 180 degrees)."""
    try:
        pin, value = msg.data.split(',')
        pin = int(pin)
        if pin in [0, 1]:
            rospy.logwarn(f"Cannot write to pin {pin} (reserved for serial communication)")
            return
        value = int(value)  # Convert value to integer (servo angles range from 0 to 180)
        if 0 <= value <= 180:
            board.digital[pin].write(value)  # Write the servo angle to the pin
            rospy.loginfo(f"Wrote servo angle {value} to pin {pin}")
        else:
            rospy.logwarn(f"Invalid servo angle {value}. Must be between 0 and 180 degrees.")
    except ValueError:
        rospy.logerr("Invalid input format. Ensure the value is a valid integer.")

# Sensor read functionality

def read_sensor_data():
    """Read sensor data from both digital and analog pins and publish to corresponding topics."""
    rate = rospy.Rate(10)  # Set the rate at which we read sensor data (10 Hz)
    while not rospy.is_shutdown() and not shutdown_flag:
        # Read data from digital pins
        for pin in range(2, len(board.digital)):  # Skip pins 0 and 1 for digital
            if board.digital[pin].mode == pyfirmata.INPUT:  # Only read from input pins
                value = board.digital[pin].read()  # Read the value from the pin
                if value is not None:
                    pub = rospy.Publisher(f"/sensor_{pin}", Float32, queue_size=10)
                    pub.publish(value)  # Publish the value to a ROS topic

        # Read data from analog pins
        for pin in range(0, len(board.analog)):  # Check all analog pins
            value = board.analog[pin].read()  # Read the analog value
            if value is not None:
                pub = rospy.Publisher(f"/sensor_A{pin}", Float32, queue_size=10)
                pub.publish(value)  # Publish the value to a ROS topic
        
        rate.sleep()  # Sleep for the set rate to maintain the loop

def arduino_node():
    """Main function to initialize the Arduino node."""
    global board, shutdown_flag

    rospy.init_node('Arduino', anonymous=False)  # Initialize the ROS node

    port = rospy.get_param('~port', '/dev/ttyUSB0')  # Use the default port
    rospy.logwarn(f"No port specified, defaulting to {port}")

    rospy.loginfo(f"Connecting to Arduino on port {port}...")

    board = pyfirmata.Arduino(port)  # Connect to the Arduino board

    # Start pyFirmata iterator to continuously read data
    it = pyfirmata.util.Iterator(board)
    it.start()

    # ROS Subscribers to listen to commands
    rospy.Subscriber("pin_mode", String, pin_mode_callback)
    rospy.Subscriber("digital_write", String, digital_write_callback)
    rospy.Subscriber("servo_write", String, servo_write_callback)
    rospy.Subscriber("analog_write", String, analog_write_callback)  # For PWM

    # Start the thread for sensor reading
    sensor_thread = threading.Thread(target=read_sensor_data)
    sensor_thread.start()

    rospy.loginfo("Arduino node ready to receive commands.")

    # Main loop
    rate = rospy.Rate(10)  # Set the loop rate (10 Hz)
    while not rospy.is_shutdown() and not shutdown_flag:
        rate.sleep()  # Sleep to maintain loop rate

    # Clean shutdown when rospy is shutting down
    if board is not None:
        try:
            rospy.loginfo("Shutting down Arduino connection due to rospy shutdown...")
            board.exit()  # Safely disconnect the Arduino
        except Exception as e:
            rospy.logwarn(f"Error during Arduino disconnection: {e}")

    rospy.loginfo("Arduino node shut down successfully.")

if __name__ == '__main__':
    try:
        arduino_node()  # Start the Arduino node
    except rospy.ROSInterruptException:
        pass
    finally:
        if board is not None:
            try:
                rospy.loginfo("Shutting down Arduino connection...")
                board.exit()  # Safely disconnect the Arduino
            except Exception as e:
                rospy.logwarn(f"Error during Arduino disconnection: {e}")
        rospy.loginfo("Arduino node terminated.")
