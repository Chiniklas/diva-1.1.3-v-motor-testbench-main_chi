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

class motor_commander:
    def __init__(self):
        rospy.init_node('motor_commander', anonymous = True)
        self.motorIDs = [1]
        self.robot_joint_state = np.zeros(len(self.motorIDs))

        rospy.loginfo('motor commander initialised')

        self.pub_states = []
        self.pub_commands = []
        self.sub_states = []
        for id in self.motorIDs:
            self.pub_states.append(rospy.Publisher('motor_state/'+str(id)+'/', tmotor_cmd, queue_size = 10))
            self.pub_commands.append(rospy.Publisher('motor_command/'+str(id)+'/', tmotor_cmd, queue_size = 10))
            # Also read the can responses
            self.sub_states.append(rospy.Subscriber('can_response/motor_'+str(id), tmotor_data, self.joint_state_callback, id))

        rospy.loginfo('publishers initialised')

        # Make sure that there is a subscriber to the node
        # Establish connection to the subscriber before publishing anything
        for idx in range(len(self.motorIDs)):
            while self.pub_states[idx].get_num_connections() == 0:
                print("Establishing connection with...", self.motorIDs[idx])
                rospy.Rate(50).sleep()
                pass

    def joint_state_callback(self, data, id):
        if id == 1:
            self.robot_joint_state[0] = data.position

    def armMotor(self, idx):
        cmd = tmotor_cmd()
        cmd.status = True
        # self.pub_states[idx].publish(cmd)
        # # Set kp and kd to 0
        # cmd = tmotor_cmd()
        cmd.setzero = False
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        self.pub_states[idx].publish(cmd)
        rospy.loginfo('motor armed')
        time.sleep(0.3)

    def setZero(self, idx):
        cmd = tmotor_cmd()
        cmd.status = True
        cmd.setzero = True
        # self.pub_states[idx].publish(cmd)
        # # Then immediately send the command to move to zero
        # cmd = tmotor_cmd()
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        self.pub_commands[idx].publish(cmd)

    def disarmMotor(self, idx):
        cmd = tmotor_cmd()
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        # self.pub_states[idx].publish(cmd)
        # cmd = tmotor_cmd()
        cmd.status = False
        cmd.setzero = False
        self.pub_states[idx].publish(cmd)
        rospy.loginfo('motor disarmed')
        time.sleep(0.3)

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
        self.pub_commands[idx].publish(cmd)
        rospy.loginfo('Angle Set')
        

    def setCommand(self, idx, angle, velocity, kp, kd):
        # !!! Since the motor command is now a different topic, we
        # might want to use a different mesgs to set the control mode etc.
        cmd = tmotor_cmd()
        cmd.status = True
        cmd.setzero = False
        # The input angle is in degrees
        cmd.position = angle
        cmd.velocity = velocity
        cmd.torque = 0
        # This ensures that we are in the position only mode
        cmd.kp = kp # 20
        cmd.kd = kd # 4
        self.pub_commands[idx].publish(cmd)

    def armAll(self):
        for idx in range(len(self.motorIDs)):
            self.armMotor(idx)
            time.sleep(0.3)

    def disarmAll(self):
        for idx in range(len(self.motorIDs)):
            self.disarmMotor(idx)
            time.sleep(0.3)

    def setZeroAll(self):
        for idx in range(len(self.motorIDs)):
            self.setZero(idx)
            time.sleep(0.3)

    def holdCurrentPose(self):
        # First arm all motors to update the joint state
        self.armAll()
        # get the current joint pose of the robot
        rospy.loginfo("Holding state: {}".format(self.robot_joint_state))
        # move the robot to its current pose so that it holds
        hold_kp = 100
        hold_kd = 0
        for ii in range(len(self.robot_joint_state)):
            self.setAngle(ii, self.robot_joint_state[ii],hold_kp,hold_kd)
            time.sleep(0.3)

    def readCurrentPassivePositions(self):
        # disarm twice to update the current joint position
        self.disarmAll()
        self.disarmAll()
        # print the joint state
        rospy.loginfo("{}".format(self.robot_joint_state))
        print(self.robot_joint_state)


# Class for interpolating positions and velocities
# Inherits the motor_commander class to send commands
class trajTracker(motor_commander):
    def __init__(self):
        # initate the motor_commander
        super().__init__()
        # in seconds
        self.tf = 0
        self.control_frequency = 0.1
        # desired angle in degrees
        self.qd = np.array([0])
        # initial angle
        self.qi = np.array([0])
        # get desired and final velocities
        self.vi = np.array([0])
        self.vd = np.array([0])
        self.interpolation_method = "linear"
        # set current real-time
        self.t0 = 0
        # save coeffs for cubic
        self.cc = np.empty(4)
        # Setup container to track open loop trajectories loaded
        self.generated_traj = {}

    # Pass t0 when the loop started and tc the current time
    def get_ref_pos(self, t0, tc):
        tn = tc-t0
        if self.interpolation_method == "linear":
            theta = self.qi*(1-tn/self.tf)+self.qd*tn/self.tf
            
            ## set every velocity to zero?? that could cause vibration
            dtheta = np.zeros(len(self.motorIDs))
            return theta, dtheta
        # use cubic interpolation for moving between positions to use velocity
        if self.interpolation_method == "cubic":
            # modify this to assert
            # if self.cc.any() == 0:
            #     print(self.cc)
            #     raise Exception("Cubic requires interpolation coeffs.")
            # else:
            # print(self.cc)
            theta = self.cc[0]*tn**3+self.cc[1]*tn**2+self.cc[2]*tn+self.cc[3]
            dtheta = 3*self.cc[0]*tn**2++2*self.cc[1]*tn+self.cc[2]
            return theta, dtheta
        if self.interpolation_method == "cubic_pos":
            theta = self.cc[0]*tn**3+self.cc[1]*tn**2+self.cc[2]*tn+self.cc[3]
            dtheta = np.zeros(len(self.motorIDs))
            return theta, dtheta
        else:
            pass

    def compute_cc(self):
        self.cc[0] = (-2*(self.qd-self.qi)-self.tf*(self.vd+self.vi))/self.tf**3
        self.cc[1] = -(-3*(self.qd-self.qi)+self.tf*(self.vd+2*self.vi))/self.tf**2
        self.cc[2] = self.vi
        self.cc[3] = self.qi

    # Generate the required trajectory from the save values and dt value
    def generate_trajectory(self, N):
        t_list = np.linspace(self.t0, self.tf, N)
        pos_list = []
        vel_list = []
        if self.interpolation_method == "linear":
            [pos_list.append(self.qi*(1-tn/self.tf)+self.qd*tn/self.tf) for tn in t_list]
            [vel_list.append(np.zeros(len(self.motorIDs))) for tn in t_list]
            self.generated_traj['time'] = t_list
            self.generated_traj['des_pos'] = np.array(pos_list)
            self.generated_traj['des_vel'] = np.array(vel_list)

        # use cubic interpolation for moving between positions to use velocity
        if self.interpolation_method == "cubic":
            t_list = np.linspace(self.t0, self.tf, N)
            [pos_list.append(self.cc[0]*tn**3+self.cc[1]*tn**2+self.cc[2]*tn+self.cc[3]) for tn in t_list]
            [vel_list.append(3*self.cc[0]*tn**2++2*self.cc[1]*tn+self.cc[2]) for tn in t_list]
            self.generated_traj['time'] = t_list
            self.generated_traj['des_pos'] = np.array(pos_list)
            self.generated_traj['des_vel'] = np.array(vel_list)


        if self.interpolation_method == "cubic_pos":
            t_list = np.linspace(self.t0, self.tf, N)
            [pos_list.append(self.cc[0]*tn**3+self.cc[1]*tn**2+self.cc[2]*tn+self.cc[3]) for tn in t_list]
            [vel_list.append(np.zeros(len(self.motorIDs))) for tn in t_list]
            self.generated_traj['time'] = t_list
            self.generated_traj['des_pos'] = np.array(pos_list)
            self.generated_traj['des_vel'] = np.array(vel_list)
        else:
            pass


    def track_trajectory(self):
        # Read current time
        self.t0 = time.time()
        tc = time.time()-self.t0
        while self.tf >= tc:
            # ideally compute qd in each id loop because we're sleeping a lot
            qr, dqr = self.get_ref_pos(self.t0, time.time())
            # print(np.array(qr), np.array(dqr))
            for idx in range(len(self.motorIDs)):
                # self.setCommand(idx, qr[idx], dqr[idx], 20, 4)
                kp = 20
                kd = 4
                self.setCommand(idx, qr, dqr, kp, kd)
                time.sleep(self.control_frequency)
                tc = time.time()-self.t0
                print(tc, qr, dqr)


if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)

    # Initiate the motor commander
    mc = motor_commander()
    # qr = np.array([0.1])
    # for idx in range(len(mc.motorIDs)):
    #     # mc.setAngle(idx, qr[idx], 20, 4)
    #     # mc.setVelocity(idx, qr[idx], 20, 4)
    #     pass
    #
    # # # Use code below for position control loop
    # # # Initiate the trajecotry trajTracker which already inherits the controller
    tt = trajTracker()
    mc.readCurrentPassivePositions()
    # Set the parameters for the tracker
    # step-1
    thetaival = 0
    thetafval = 180
    tt.qi = np.array([thetaival])
    tt.qd = np.array([thetafval])
    tt.tf = 3
    tt.vd = 0
    tt.vi = 0
    tt.control_frequency = 0.01
    
    # For cubic interpolation
    tt.interpolation_method = "cubic"
    # compute coeffs
    if tt.interpolation_method == "cubic" or tt.interpolation_method == "cubic_pos":
        tt.compute_cc()
    # print(tt.cc)
    
    # generate open-loop trajectory
    #--------------------------------------
    # mc.armMotor(0)
    # # mc.setZero(0)
    # time.sleep(2)
    
    # tt.interpolation_method = "linear"
    # N = 50
    # tt.generate_trajectory(N)
    # print(tt.generated_traj)
    # for ii in range(N):
    #     # print(tt.generated_traj['time'][ii], tt.generated_traj['des_pos'][ii],tt.generated_traj['des_vel'][ii])
    #     mc.setCommand(0,tt.generated_traj['des_pos'][ii],tt.generated_traj['des_vel'][ii],12,1)
    #     time.sleep(tt.control_frequency)
    #     # mc.readCurrentPassivePositions()
    #     # print(mc.robot_joint_state)
    # time.sleep(0.5)
    
    # mc.disarmMotor(0)
    # mc.readCurrentPassivePositions()
    #-------------------------------------------
    mc.armMotor(0)
    time.sleep(2)
    tt.interpolation_method = "cubic_pos"
    N = 50
    tt.generate_trajectory(N)
    print(tt.generated_traj)
    for ii in range(N):
        print(tt.generated_traj['time'][ii], tt.generated_traj['des_pos'][ii],tt.generated_traj['des_vel'][ii])
        mc.setCommand(0,tt.generated_traj['des_pos'][ii],tt.generated_traj['des_vel'][ii],20,1)
        time.sleep(tt.control_frequency)
    
    time.sleep(1)
    mc.disarmMotor(0)
    mc.readCurrentPassivePositions()
    #-------------------------------------------------------
    # print(mc.robot_joint_state)
    # mc.armMotor(0)
    # time.sleep(1)
    # mc.setZero(0)
    # time.sleep(1)
    # mc.disarmMotor(0)
    # mc.holdCurrentPose()
    # mc.armAll()
    # mc.setZeroAll()
    # time.sleep(1)
    #
    # thetaival = 0
    # thetafval = 180
    # tt.qi = np.array([thetaival])
    # tt.qd = np.array([thetafval])
    # tt.tf = 2
    # tt.vd = 0
    # tt.vi = 0
    # tt.control_frequency = 0.01
    # # For cubic interpolation
    # tt.interpolation_method = "cubic"
    # # compute coeffs
    # if tt.interpolation_method == "cubic" or tt.interpolation_method == "cubic_pos":
    #     tt.compute_cc()
    #
    # # Now move the motor
    # tt.track_trajectory()
    # print("Current state: ", mc.robot_joint_state)
    # time.sleep(3)
    #
    # print(mc.robot_joint_state)
    #
    # # # step-2
    # tt.qi = np.array([thetafval])
    # tt.qd = np.array([thetaival])
    # tt.interpolation_method = "cubic"
    # if tt.interpolation_method == "cubic" or tt.interpolation_method == "cubic_pos":
    #     tt.compute_cc()
    #
    # tt.track_trajectory()
    # print("Current state: ", mc.robot_joint_state)

    # mc.disarmAll()


    # Running in pure velocity control mode
    # mc.setCommand(0, 0, 10, 0, 0.5)
    # time.sleep(10)
    # mc.setCommand(0, 0, 0, 20, 0)

    # Now lets try what DFKI is doing, i,e., running an open loop controller with saved position and velocities and see if that is smooth and predicatable
    # For which we need a function to generate the open loop trajectory given initial and final positions and the dt between them
