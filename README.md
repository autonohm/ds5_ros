# ds5_ros

A ROS1 Node to integrate the DualSense 5 (PS5) Controller into the ROS ecosystem, including feedback messages to set leds, adaptive triggers etc.

# Installing

Install the hidapi.

```bash
sudo apt install libhidapi-dev
```
Install the package from [pypi](https://pypi.org/project/pydualsense/).

```bash
pip install pydualsense
```

To run the package node
```bash
sudo su root
```

OR

```bash
sudo chmod -R 777 /dev
```


# Usage

Initialize the controller

```python
from pydualsense import *

ds = pydualsense() # open controller
ds.init() # initialize controller
```

## LED

Set color RGB. RGB are integer values and in range 0-255.

```python
ds.light.setColor(255, 0, 0)
```
## Rumble effect

Control the rumble effect of left motor or right motor. Intensity is in range 0-255.

```python
ds.setLeftMotor(255)
```
## Adaptive Trigger

Set trigger mode for the buttons and control the intensity.

```python
ds.TriggerR.setMode(TriggerModes.Rigid)
ds.TriggerR.setForce(1, 255)
```
## Other

Other effects and enumerations could be found under pydualsense.py and enums.py

# Topic

## Publisher

Control robot:
- Topic: cmd_vel
- Message type: [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)

Control gripper:
- Topic: ???
- Message type: ???

## Subscriber

Receive feedback from roboter arm:
- Topic: joy/set_feedback
- Message type: [JoyFeedbackArray](http://docs.ros.org/en/api/sensor_msgs/html/msg/JoyFeedbackArray.html)

# Button Mapping

BUTTON_ACTION_CROSS     = ds.state.cross 

BUTTON_ACTION_CIRCLE     = ds.state.circle

BUTTON_ACTION_TRIANGLE     = ds.state.triangle

BUTTON_ACTION_SQUARE     = ds.state.square

BUTTON_CROSS_UP     = ds.state.DpadUp

BUTTON_CROSS_DOWN     = ds.state.DpadDown

BUTTON_CROSS_LEFT     = ds.state.DpadLeft

BUTTON_CROSS_RIGHT     = ds.state.DpadRight

AXIS_REAR_LEFT_2    = ds.state.L2 (0 - 255)

AXIS_REAR_RIGHT_2    = ds.state.R2 (0 - 255)

BUTTON_REAR_LEFT_2    = ds.state.L2Btn

BUTTON_REAR_RIGHT_2    = ds.state.R2Btn

BUTTON_REAR_LEFT_1    = ds.state.L1

BUTTON_REAR_RIGHT_1    = ds.state.R1


//Problem (Sometimes not zero by uncontrolled Joy)

AXIS_STICK_LEFT_RIGHTWARDS     = ds.state.LX (-127 - 128)

AXIS_STICK_LEFT_DOWNWARDS    = ds.state.LY (-127 - 128)

AXIS_STICK_RIGHT_RIGHTWARDS    = ds.state.RX (-127 - 128)

AXIS_STICK_RIGHT_DOWNWARDS    = ds.state.RY (-127 - 128)

Other controll function like touch pad and option buttons can be found in pydualsense.py

# Credits

[https://github.com/flok/pydualsense](https://github.com/flok/pydualsense)
