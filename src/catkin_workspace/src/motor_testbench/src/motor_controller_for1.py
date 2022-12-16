#!/usr/bin/env python3
import rospy
from motor_testbench.msg import *
import signal
import math
import time
import numpy as np
import csv

def exit_gracefully(signum, frame):
    signal.signal(signal.SIGINT, original_sigint)

    sys.exit(1)

    signal.signal(signal.SIGINT, exit_gracefully)
    
class motor_controller:
    def __init__(self) -> None:
        # motor_controller initialization
        rospy.init_node('motor_controller')
        self.motorIDs = 1
        rospy.loginfo('motor controller initialised')
        # self.robot_joint_state = np.zeros(len(self.motorIDs))
        self.pub_states = rospy.Publisher('motor_state/1',tmotor_cmd,queue_size=10)
        self.pub_commands = rospy.Publisher('motor_command/1',tmotor_cmd,queue_size=10)
        
        self.sub_states = rospy.Subscriber('can_response/motor_'+str(self.motorIDs), tmotor_data, self.joint_state_callback, self.motorIDs)
        
        rospy.loginfo('publishers initialised')
        
        while self.pub_states.get_num_connections() == 0:
                print("Establishing connection with...", self.motorIDs)
                rospy.Rate(50).sleep()
                pass
         
    def joint_state_callback(self, data, id):
        if id == 1:
            self.robot_joint_state = data.position
        
    def armMotor(self,idx):
        cmd = tmotor_cmd()
        cmd.status = True
        cmd.setzero = False
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        self.pub_states.publish(cmd)
        rospy.loginfo('motor armed')
        time.sleep(0.3)
    
    def disarmMotor(self,idx):
        cmd = tmotor_cmd()
        cmd.status = False
        cmd.setzero = False
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        self.pub_states.publish(cmd)
        rospy.loginfo('motor disarmed')
        time.sleep(0.3)

    def setZero(self, idx):
        # not tested yet
        cmd = tmotor_cmd()
        cmd.status = True
        cmd.setzero = True
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        self.pub_commands[idx].publish(cmd)
        
    def setAngle(self, idx, angle, kp, kd):
        # !!! Since the motor command is now a different topic, we
        # might want to use a different mesgs to set the control mode etc.
        cmd = tmotor_cmd()
        cmd.status = True
        cmd.setzero = False
        # The input angle is in degrees
        cmd.position = angle
        cmd.velocity = 0
        cmd.torque = 0
        # This ensures that we are in the position only mode
        cmd.kp = kp # 20
        cmd.kd = kd # 4
        self.pub_commands.publish(cmd)
        rospy.loginfo('Angle Set')
        # time.sleep(0.1)
        
    def setCommand(self, idx, angle, velocity, kp, kd):
        # !!! Since the motor command is now a different topic, we
        # might want to use a different mesgs to set the control mode etc.
        cmd = tmotor_cmd()
        # The input angle is in degrees
        cmd.position = angle
        cmd.velocity = velocity
        cmd.torque = 0
        # This ensures that we are in the position only mode
        cmd.kp = kp # 20
        cmd.kd = kd # 4
        self.pub_commands.publish(cmd)
        rospy.loginfo('command sent')
        # time.sleep(0.1)   
        
    def setVelocity(self,idx,velocity,kd):
        # this is the low level control command for velocity control
        # the unit of velocity is degree/s
        # 1 rpm = 6 degree/s
        cmd = tmotor_cmd()
        cmd.position = 0
        cmd.velocity = velocity
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = kd
        self.pub_commands.publish(cmd)
        rospy.loginfo('command sent')
        
    def readCurrentPassivePositions(self):
        # disarm twice to update the current joint position
        # self.disarmAll()
        # self.disarmAll()
        # print the joint state
        rospy.loginfo("{}".format(self.robot_joint_state))
        print(self.robot_joint_state)
    
if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    
    mc = motor_controller()
    rospy.loginfo('start test')
    mc.armMotor(1)
    # simple test
    # mc.setAngle(1,50,8,1)
    # mc.setAngle(1,0,8,1)
    
    # linear interpolation using theta for position control loop
    step = 5
    start = 0
    end = 90
    for i in range(start,end+step,step):
        rospy.loginfo(i)
        # mc.setAngle(1,i,8,1)
        mc.setCommand(1,i,1,5,0.25)
        
        # control rate = 0.01
        time.sleep(0.01)
        mc.readCurrentPassivePositions()
    time.sleep(1)
    
    mc.disarmMotor(1)
    mc.readCurrentPassivePositions()
    
    # velocity control
    # mc.armMotor(1)
    # mc.setVelocity(1,360,1)
    # time.sleep(10)
    # mc.disarmMotor(1)
    
    
    # mc.disarmMotor(1)
    # mc.readCurrentPassivePositions()