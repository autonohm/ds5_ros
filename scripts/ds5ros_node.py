#!/usr/bin/env python3

#ManhLinhPhan


import rospy
import math
from sensor_msgs.msg import JoyFeedbackArray, Joy

from pydualsense import *

class Ds5Ros():
    def __init__(self):
        # create dualsense
        self.dualsense = pydualsense()
        # find device and initialize
        self.dualsense.init()

        self.noderate = rospy.get_param("noderate", 50.0)

        # receive feedback from robot_joy_control node 
        self.joy_sub_topic = rospy.get_param("joy_sub", "joy/set_feedback")
        # send signal to robot_joy_control node
        self.joy_pub_topic = rospy.get_param("joy_pub", "joy")

        self.joy_sub = rospy.Subscriber(self.joy_sub_topic, JoyFeedbackArray, self.set_feedback, queue_size= 10)
        self.joy_pub = rospy.Publisher(self.joy_pub_topic, Joy, queue_size = 1)

    '''
    JoyFeedback Struct
    TYPE_LED = 0 | TYPE_RUMBLE = 1 (use also for feedback on rear button) | TYPE_BUZZER = 2
    type    : above
    id      : 0 = Left Motor Rumble | 1 = Right Motor Rumble | 2 = L2 feedback Trigger | 3 = R2 feedback Trigger
    Using combination of type and id to get the result
    intensity:  
    '''

    def set_feedback(self, msg):
        print(msg)
        for feedback in msg.array: #not iterable
            #set continuous force on left rear button
            #change this part if received message intensity is from 0 - 1.0
            if feedback.intensity > 255 and feedback.intensity <0:
                raise Exception('intensity muss in range 0 - 255')
                continue
            
            #intensity muss be an integer
            if feedback.type == 1 and feedback.id == 0:
                pass #add effect later
            elif feedback.type == 1 and feedback.id == 1:
                pass #add effect later
            elif feedback.type == 1 and feedback.id == 2:
                self.dualsense.triggerL.setMode(TriggerModes.Rigid)
                self.dualsense.triggerL.setForce(1, int(feedback.intensity))
            elif feedback.type == 1 and feedback.id == 3:
                self.dualsense.triggerR.setMode(TriggerModes.Rigid)
                self.dualsense.triggerR.setForce(1, int(feedback.intensity) )

    def joy_publish(self):
        joy_msg = Joy()

        print("in joy_publish")
        
        for i in range(18):
            joy_msg.buttons.append(0)
        # Buttons mapping
        joy_msg.buttons[0] = self.dualsense.state.cross
        joy_msg.buttons[1] = self.dualsense.state.circle
        joy_msg.buttons[2] = self.dualsense.state.triangle
        joy_msg.buttons[3] = self.dualsense.state.square

        joy_msg.buttons[13] = self.dualsense.state.DpadUp
        joy_msg.buttons[16] = self.dualsense.state.DpadRight
        joy_msg.buttons[14] = self.dualsense.state.DpadDown
        joy_msg.buttons[15] = self.dualsense.state.DpadLeft

        joy_msg.buttons[6] = self.dualsense.state.L2Btn
        joy_msg.buttons[7] = self.dualsense.state.R2Btn
        joy_msg.buttons[4] = self.dualsense.state.L1
        joy_msg.buttons[5] = self.dualsense.state.R1
        joy_msg.buttons[8] = self.dualsense.state.L3
        joy_msg.buttons[9] = self.dualsense.state.R3
        
        joy_msg.buttons[10] = self.dualsense.state.ps
        joy_msg.buttons[11] = self.dualsense.state.share
        joy_msg.buttons[12] = self.dualsense.state.options

        joy_msg.buttons[17] = self.dualsense.state.touchBtn

        # Axes mapping
        for i in range(6):
            joy_msg.axes.append(0)

        joy_msg.axes[0] = -1 * self.dualsense.state.LX + 1  #Leftward (-128 -> 127, default ~0)
        joy_msg.axes[1] = -1 * self.dualsense.state.LY + 1  #Upward (-128 -> 127, default ~0)
        joy_msg.axes[2] = -1 * self.dualsense.state.RX + 1  #Leftward (-128 -> 127, default ~0)
        joy_msg.axes[3] = -1 * self.dualsense.state.RY + 1  #Upward (-128 -> 127, default ~0)
        joy_msg.axes[4] = self.dualsense.state.L2           #PushDown (0 -> 255, default = 0)
        joy_msg.axes[5] = self.dualsense.state.R2           #PushDown (0 -> 255, default = 0)

        self.joy_pub.publish(joy_msg)

    def main_loop(self):
        rate = rospy.Rate(self.noderate)
        while not rospy.is_shutdown():
            self.joy_publish()
            rate.sleep()

def main():
    rospy.init_node("ds5ros_node")
    ds5 = Ds5Ros()
    ds5.main_loop()

if __name__== '__main__':
    main()
   