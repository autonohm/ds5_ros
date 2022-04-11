#!/usr/bin/env python3

#ManhLinhPhan

import rospy
import math
from sensor_msgs.msg import JoyFeedbackArray, JointState
from geometry_msgs.msg import Twist

from pydualsense import *


class Ps5Joy():
    def __init__(self):
        # create dualsense
        self.dualsense = pydualsense()
        # find device and initialize
        self.dualsense.init()

        self.noderate = rospy.get_param("noderate", 50.0)
        
        self.arm_sub_topic = rospy.get_param("arm_sub_topic", "joy/set_feedback")
        #arm_pub_topic = rospy.get_param("arm_pub_topic", ??? Motor topic)
        self.twist_pub_topic = rospy.get_param("twist_pub_topic", "cmd_vel")

        self.scale_linear_x = rospy.get_param("scale_linear_x", 1.0)
        self.scale_linear_y = rospy.get_param("scale_linear_y", 1.0)
        self.scale_angular_z = rospy.get_param("scale_angular_z", 1.0)
        self.scale_precise_movement = rospy.get_param("scale_precise_movement", 0.1)

        #self.scale_arm = rospy.get_param("")
        #self.scale_arm_feedback = rospy.get_param("")

        self.arm_sub = rospy.Subscriber(self.arm_sub_topic, JoyFeedbackArray, self.set_feedback)
        #self.arm_pub = rospy.Publisher(arm_pub_topic, Joy,queue_size= 1)
        self.twist_pub = rospy.Publisher(self.twist_pub_topic, Twist, queue_size = 10)

        rospy.loginfo("robot_joy_control: PARAMETER")
        rospy.loginfo("robot_joy_control: noderate = " + str(self.noderate))
        rospy.loginfo("robot_joy_control: arm_sub_topic = " + str(self.arm_sub_topic))
        # rospy.loginfo("robot_joy_control: arm_pub_topic = " + str(self.arm_pub_topic))
        rospy.loginfo("robot_joy_control: twist_pub_topic = " + str(self.twist_pub_topic))
        rospy.loginfo("robot_joy_control: scale_linear_x = " + str(self.scale_linear_x))
        rospy.loginfo("robot_joy_control: scale_linear_y = " + str(self.scale_linear_y))
        rospy.loginfo("robot_joy_control: scale_angular_z = " + str(self.scale_angular_z))
        rospy.loginfo("robot_joy_control: scale_precies_movement = " + str(self.scale_precise_movement))


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
            
    def twist_control(self):
        cmd_vel = Twist()
        #normal control while R1 is pressed
        if(self.dualsense.state.R1):
            # -1 because of Right and Down is positive
            cmd_vel.linear.x = self.scale_linear_x * -1 *self.dualsense.state.LY
            cmd_vel.linear.y = self.scale_linear_y * -1 *self.dualsense.state.LX
            cmd_vel.angular.z = self.scale_angular_z * -1 *self.dualsense.state.RX
            #publish cmd_vel
            self.twist_pub.publish(cmd_vel)
        #precise control while L1 is pressed
        elif(self.dualsense.state.L1):
            cmd_vel.linear.x = self.scale_precise_movement  * (self.dualsense.state.DpadUp - self.dualsense.state.DpadDown)
            cmd_vel.linear.y = self.scale_precise_movement  * (self.dualsense.state.DpadLeft - self.dualsense.state.DpadRight)
            cmd_vel.angular.z = self.scale_precise_movement * math.pi * (self.dualsense.state.square - self.dualsense.state.circle)
            #publish cmd_vel
            self.twist_pub.publish(cmd_vel)
        else:
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.angular.z = 0
            #
            #self.twist_pub.publish(cmd_vel)
        #Adjustment of Precise Movement Params
        timeNow = rospy.get_rostime()

        if(timeNow > debounce_timer + rospy.Duration(0.2)):
            if(self.dualsense.state.triangle):
                self.scale_precise_movement += 0.05
                rospy.loginfo("Precise Movement Scale - Up - NewVal = %f", self.scale_precise_movement)
                debounce_timer = timeNow
            if(self.dualsense.state.cross):
                self.scale_precise_movement -= 0.05
                rospy.loginfo("Precise Movement Scale - Up - NewVal = %f", self.scale_precise_movement)
                debounce_timer = timeNow

    #do it later when have topic, msg from
    def arm_control(self):
        pass

    def main_loop(self):
        rate = rospy.Rate(self.noderate)
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(10, "robot_joy_control: Use Triangle and Cross to Adjust Precise Movement param")
            self.twist_control()
            self.arm_control()
            rate.sleep()

def main():
    rospy.init_node("ps5joy_node")
    ps5 = Ps5Joy()
    ps5.main_loop()

if __name__ =='__main__':
    main()
    