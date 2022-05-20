#!/usr/bin/env python3

#ManhLinhPhan


import rospy
import math
from sensor_msgs.msg import JoyFeedbackArray, Joy

from pydualsense import *

class Ds5Ros():
    def __init__(self):

        self.logging_prefix = "DS5_ROS: "
        
        # 0 is startup, 1 is running
        self.node_state = 0

        # create dualsense
        self.dualsense = pydualsense()


        self.noderate = rospy.get_param("noderate", 50.0)

        # receive feedback from robot_joy_control node 
        self.joy_sub_topic = rospy.get_param("joy_sub", "joy/set_feedback")
        # send signal to robot_joy_control node
        self.joy_pub_topic = rospy.get_param("joy_pub", "joy")

        # controller is not straight zero for the axis.. prevent tiny robot movements 
        self.deadzone = rospy.get_param("deadzone", 0.05)


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
        #print(msg)
        for feedback in msg.array: #not iterable
            #set continuous force on left rear button
            #change this part if received message intensity is from 0 - 1.0
            if feedback.intensity > 1.0 or feedback.intensity <0.0:
                rospy.logerr(self.logging_prefix + 'intensity must be in range 0.0 - 1.0')
                continue
            else:
                feedback.intensity = feedback.intensity * 255.0
            
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

        #print("in joy_publish")
        
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

        # change value by Axes from 0.0 -> 1.0
        joy_msg.axes[0] = (-1 * self.dualsense.state.LX )/128.0         #Leftward   (-1.0 -> 1.0, default ~0.0)
        joy_msg.axes[1] = (-1 * self.dualsense.state.LY )/128.0         #Upward     (-1.0 -> 1.0, default ~0.0)
        joy_msg.axes[2] = (-1 * self.dualsense.state.RX )/128.0         #Leftward   (-1.0 -> 1.0, default ~0.0)
        joy_msg.axes[3] = (-1 * self.dualsense.state.RY )/128.0         #Upward     (-1.0 -> 1.0, default ~0.0)
        joy_msg.axes[4] = self.dualsense.state.L2 /255.0                #PushDown   (0.0 -> 1.0, default = 0)
        joy_msg.axes[5] = self.dualsense.state.R2 /255.0                #PushDown   (0.0 -> 1.0, default = 0)

        for val in range(6):
            if(abs(joy_msg.axes[val]) < self.deadzone):
                joy_msg.axes[val] = 0.0

        self.joy_pub.publish(joy_msg)

    def main_loop(self):
        rate = rospy.Rate(self.noderate)

        ### initialize controller if possible, else wait
        
        # find device and initialize
        while not rospy.is_shutdown():
            if self.node_state is 0:
                try:
                    self.dualsense.init()
                except:
                    rospy.logwarn_throttle(2, "Cannot initialize controller!") 
                else:
                    # set rgb led to green
                    self.node_state = 1
            
            elif self.node_state is 1:
                rospy.loginfo_throttle(2, "DS5_Ros is alive!") 
                # if error -> node_state = 0
                try:
                    self.joy_publish()
                except IOError:
                    rospy.logerr("Lost connection! Go back to init!")
                    self.node_state = 0
            
            rate.sleep()
        self.dualsense.close()

if __name__== '__main__':
    rospy.init_node("ds5ros_node")
    ds5 = Ds5Ros()
    ds5.main_loop()
   
