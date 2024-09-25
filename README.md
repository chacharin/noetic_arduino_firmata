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
git clone https://github.com/chacharin/noetic_arduino_firmata.git
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
rosrun noetic_arduino_firmata arduino_node.py
```
This will launch the ROS node that interacts with the Arduino.
