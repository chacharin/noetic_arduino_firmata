# noetic_arduino_firmata
The easy way for ros noetic developer to control standard firmata on an Arduino board.
This ROS Noetic package allows interaction with an Arduino board using the standard Firmata protocol. 
Below are the detailed steps to set up the package on your computer.

## Installation Instructions

### Step 1: Navigate to the `src` Directory of Your Workspace
Open a terminal and navigate to the `src` directory of your existing `catkin_ws` ROS workspace:

```bash
cd ~/catkin_ws/src
```

### Step 2: Clone the Repository
Clone the noetic_arduino_firmata package from GitHub:

```bash
git clone https://github.com/chacharin/noetic_uno_firmata.git
```
This will download the package into your catkin_ws/src directory.

### Step 3: Install Required Dependencies

Ensure that all necessary dependencies are installed by running the following command:

```bash
sudo apt-get install ros-noetic-rospy ros-noetic-std-msgs python3-pyfirmata
```
This installs the required ROS packages and the Python pyfirmata library.

### Step 4: Build the Workspace
Go back to your ROS workspace and build the package:

```bash
cd ~/catkin_ws
catkin_make
```
This command will compile the noetic_arduino_firmata package and all other packages in your workspace.

### Step 5: Source the Workspace

Source the workspace so that ROS can recognize the new package:

```bash
source ~/catkin_ws/devel/setup.bash
```

To automatically source the workspace in all future terminal sessions, add this line to your .bashrc:
bash
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 6: Run the Node

Now you can run the arduino_node.py script using the following command:

```bash
rosrun noetic_uno_firmata arduino_node.py
```
This will launch the ROS node that interacts with the Arduino.



## How to Use

### Step 1: Launch the Arduino Node

Connect your Arduino to your computer. To start the node, specify the correct port for your Arduino. 


By default, the port is set to `/dev/ttyUSB0`, but you can change this to match your setup (e.g., `/dev/ttyACM0`):

```bash
rosrun noetic_arduino_firmata arduino_node.py _port:=/dev/ttyACM0
```

Replace `/dev/ttyACM0` with the correct serial port for your Arduino.
To find the correct port for your Arduino, you can use the following command to list available serial ports:

```bash
ls /dev/tty*
```

### Step 2: Control the Arduino Using ROS Topics

Once the node is running, you can control and interact with the Arduino using the following ROS topics:

#### Set Pin Mode

You can set the pin mode (input, output, PWM, or servo) using the `pin_mode` topic:

- For digital pin output:
  
```bash
rostopic pub -1 /pin_mode std_msgs/String "[pin],output"
```

- For setting a digital pin as PWM:

```bash
rostopic pub -1 /pin_mode std_msgs/String "[pin],pwm"
```

- For configuring a pin for a servo:

```bash
rostopic pub -1 /pin_mode std_msgs/String "[pin],servo"
```

- For setting a pin as input (e.g., to read sensor data):

```bash
rostopic pub -1 /pin_mode std_msgs/String "[pin],input"
```

#### Write to Digital or Analog Pins

- To write a digital value to a pin (e.g., turning on an LED):

```bash
rostopic pub -1 /digital_write std_msgs/String "[pin],1"
```

- To write a PWM value to a pin (0.0 to 1.0):

```bash
rostopic pub -1 /analog_write std_msgs/String "[pin],0.5"
```

- To control a servo by setting its angle (0 to 180 degrees):

```bash
rostopic pub -1 /servo_write std_msgs/String "[pin],90"
```

#### Reading Sensor Data

To read input from digital or analog pins, first set the pin as an input:

For a digital pin (e.g., Digital Pin 9):

```bash
rostopic pub -1 /pin_mode std_msgs/String "9,input"
```

Then, use `rostopic echo` to read the values being published from the sensor on the specified pin:

```bash
rostopic echo /sensor_9
```
For an analog pin (e.g., Analog Pin A0):

  ```bash
  rostopic pub -1 /pin_mode std_msgs/String "A0,input"
  ```
Then, read the values being published from the analog sensor on Pin A0:

  ```bash
  rostopic echo /sensor_A0
  ```

Analog pins are handled similarly to digital pins, with the topic names prefixed by sensor_A for each analog pin (e.g., /sensor_A1 for Analog Pin A1, /sensor_A2 for Analog Pin A2).

### Example Usage

Set pin 9 to output mode:

  ```bash
  rostopic pub -1 /pin_mode std_msgs/String "9,output"
  ```

Turn on a digital pin 9 (assuming an LED is connected):

  ```bash
  rostopic pub -1 /digital_write std_msgs/String "9,1"
  ```

Set pin 9 to PWM mode:

  ```bash
  rostopic pub -1 /pin_mode std_msgs/String "9,pwm"
  ```

Write a PWM value of 50% duty cycle to pin 9:

  ```bash
  rostopic pub -1 /analog_write std_msgs/String "9,0.5"
  ```

Configure pin 9 as a servo and set the angle to 45 degrees:

  ```bash
  rostopic pub -1 /servo_write std_msgs/String "9,45"
  ```

## Troubleshooting: Running on Ubuntu in VirtualBox

  When running the `arduino_node.py` in Ubuntu inside a VirtualBox environment, you may encounter an issue where your system freezes or becomes unresponsive. This is a known problem that can occur if the USB connection between your Arduino board and the virtual machine becomes unstable during the execution of the program.

### Solution:

  To avoid this issue, follow these steps when terminating the node:
  
  1. **Unplug the USB cable** that connects your Arduino to your computer **before** pressing `Ctrl+C` to terminate the program.
  
     This ensures that the system properly disconnects from the USB device, preventing any hangups or freezing.
  
  2. After the USB is unplugged, you can safely press `Ctrl+C` in the terminal to stop the ROS node.
  
  If you accidentally press `Ctrl+C` without unplugging the Arduino, and your system freezes, follow these steps to recover:
  
  - **Unplug the USB cable**: This should immediately allow the virtual machine to recover from the freeze or lag.
  - If the system is still unresponsive, **restart the VirtualBox machine** to regain functionality.

## Limitation: Pin Mode Switching Between Servo and PWM is Not Supported

  When using the Servo mode on a pin in Firmata, the Servo Library automatically attaches that pin to control the servo motor. However, there is a known limitation: **StandardFirmata** does not provide an algorithm or mechanism to detach the pin from Servo mode once it is attached. This limitation prevents switching the same pin from Servo mode to PWM (or any other mode) dynamically during runtime.
  
  If you attempt to set the pin to PWM after it has been configured for Servo, the pin will remain in Servo mode, and the PWM functionality will not work as expected. The current workaround to switch a pin from Servo to PWM is to reset the Arduino board and then reconnect with `arduino_node.py` to reconfigure the pin mode.
