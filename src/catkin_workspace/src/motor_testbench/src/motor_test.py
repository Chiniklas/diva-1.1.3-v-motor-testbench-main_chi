#!/usr/bin/env python3

from motor_testbench.msg import *


# import utils
# -------------------------------
# Shift it into a utils module
# -------------------------------
import signal
import sys

# !!! Need to clarify this
def exit_gracefully(signum, frame):
    signal.signal(signal.SIGINT, original_sigint)
    sys.exit(1)
    signal.signal(signal.SIGINT, exit_gracefully)

def float_to_uint(x_float, x_min, x_max, bits):
    span = x_max-x_min
    # Set software limits for the demanded variables
    if x_float<x_min:
        x_float = x_min
    if x_float>x_max:
        x_float = x_max
    # !!! The documentation uses bits+1, but I do not get why
    return int((x_float-x_min)*bits/span)


def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max-x_min
    return float((span/bits)*x_int+x_min)
# -------------------------------


import rospy
from can_msgs.msg import Frame
import math
import json

class motor_testbench:
    def __init__(self):
        # Setting up the main node
        rospy.init_node('motor_testbench')
        # Read the motorID from the node name
        nodename = rospy.get_name()
        self.motorID = int(nodename.split('_')[1])
        self.motor_active = False
        # Define a variable to hold if the motor position been set to zero changed
        # Because two consecutive commands to set and unset from zero doesnt work
        self.motor_pos_reset = False
        # Log connection info
        rospy.loginfo('motor testbench initialised')

        # A topic that where we can write the control commands
        rospy.Subscriber('motor_state/'+str(self.motorID)+'/', tmotor_cmd, self.motor_state_callback)
        rospy.Subscriber('motor_command/'+str(self.motorID)+'/', tmotor_cmd, self.motor_command_callback)
        rospy.loginfo('motor state subscriber established')
        rospy.loginfo('motor command subscriber established')
        
        # Parsed CAN messages republished here as data
        self.can_response = rospy.Publisher('can_response/motor_'+str(self.motorID), tmotor_data, queue_size=10)
        # # Initiate publishers to the CAN
        self.can_command = rospy.Publisher('sent_messages', Frame, queue_size = 10)
        # Using socketcan to read frames received from CAN
        rospy.Subscriber('received_messages', Frame, self.can_msgrecv)

        # Now disarm the motor by default whenever the motor is switched on
        self.disarmMotor()
        rospy.loginfo('Initialising motor in disarmed state')

        # Fixed for all the motors
        self.max_pos_bits = 0xFFFF # 16 bits for pos
        self.max_tau_bits = 0xFFF # 12 bits for current
        self.max_vel_bits = 0xFFF # 12 bits for vel

        # For control params
        self.min_kp = 0
        self.max_kp = 500
        self.max_kp_bits = 0xFFF

        self.min_kd = 0
        self.max_kd = 5
        self.max_kd_bits = 0xFFF

        self.max_pos = 12.5 # rad
        self.min_pos = -12.5 # rad

        # Add versions too
        motors_list = ['AK60-6', 'AK70-10', 'AK80-9']
        # motor_selected = 'AK80-9'
        motor_selected = 'AK60-6'

        # AK-60-6
        if motor_selected == motors_list[0]:
            # Setup limits here
            self.gear_ratio = 6

            # velocity
            self.min_vel = -45.0 # rad/s
            self.max_vel = 45.0 # rad/s

            # torque
            self.min_tau = -15.0
            self.max_tau = 15.0

            self.Kt = 0.068 # Nm/A

        # AK-70-10
        if motor_selected == motors_list[1]:
            # Setup limits here
            self.gear_ratio = 10

            # velocity
            self.min_vel = -50.0 # rad/s
            self.max_vel = 50.0 # rad/s

            # torque
            self.min_tau = -25.0
            self.max_tau = 25.0

            self.Kt = 0.091 # Nm/A

        # AK-80-9
        if motor_selected == motors_list[2]:
            # Setup limits here
            self.gear_ratio = 9

            # velocity
            self.min_vel = -50.0 # rad/s
            self.max_vel = 50.0 # rad/s

            # torque
            self.min_tau = -18.0
            self.max_tau = 18.0

            self.Kt = 0.091 # Nm/A
        #=================================
        rospy.spin()
        #==================================
        
    def motor_state_callback(self, data):
        msg = Frame()
        msg.id = self.motorID
        msg.is_rtr = False
        msg.is_extended = False
        msg.is_error = False
        msg.dlc = 8

        rospy.loginfo('received motor_state data')
        # Use status to arm or disarm the motors
        if data.status == True:
            if data.setzero == False:
                # # Check if the motor is already active
                # if not self.motor_active:
                # If the motor is inactive --> activate it
                msg.data = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC]
                # Set motor to be active
                self.motor_active = True
                self.can_command.publish(msg)

        # If data.status is 0, then disarm,
        if data.status == False:
            if data.setzero == False:
                # # Check if the motor is already deactivated
                # if self.motor_active:
                # Deactivate it if not deactivated already
                msg.data = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD]
                # Set motor to be deactive
                self.motor_active = False
                self.can_command.publish(msg)

        # Check if the position is reset
        # This can only be done when the status is active as per documents
        # cannot set to zero without arming, i.e., only checks if the motor is armed
        if data.setzero == True:
            if data.status == True:
                msg.data = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE]
                self.can_command.publish(msg)

    # Motor state callback is now differentiated from motor data callback
    # This way, the state variables can be safely ignored when commands are passed
    def motor_command_callback(self, data):
        cmd = Frame()
        cmd.id = self.motorID
        cmd.is_rtr = False
        cmd.is_extended = False
        cmd.is_error = False
        cmd.dlc = 8
        rospy.loginfo('received motor_command data')
        # Parse the requested motor command and send it as a CAN frame
        # Check if the motor is armed
        # THIS NEEDS TO BE TAKEN CARE!!! ONLY WORK IN DEGREES BUT SEND IN RADIANS
        p_des = math.radians(data.position)
        v_des = math.radians(data.velocity)
        # rospy.loginfo(v_des)
        tau_des = data.torque
        kp_val = data.kp
        kd_val = data.kd
        if not self.motor_active:
            raise Exception('The motor is not armed, please click "Enable" first')
        else:
            # We just need to do the reverse scaling done when the data was obtained
            # Enable software limits again here
            p_des = max(min(p_des, self.max_pos), self.min_pos)
            # pos_bits = int((p_des-self.min_pos)*self.max_pos_bits/(self.max_pos-self.min_pos))
            pos_bits = float_to_uint(p_des, self.min_pos, self.max_pos, self.max_pos_bits)
            data0 = pos_bits >> 8
            data1 = pos_bits & 0xFF
            # velocity float --> bits
            v_des = max(min(v_des, self.max_vel), self.min_vel)
            # vel_bits = int((v_des-self.min_vel)*self.max_vel_bits/(self.max_vel-self.min_vel))
            vel_bits = float_to_uint(v_des, self.min_vel, self.max_vel, self.max_vel_bits)
            data2 = vel_bits >> 4
            kp_val = max(min(kp_val, self.max_kp), self.min_kp)
            # kp_bits = int((kp_val-self.min_kp)*self.max_kp_bits/(self.max_kp-self.min_kp))
            kp_bits = float_to_uint(kp_val, self.min_kp, self.max_kp, self.max_kp_bits)
            data3 = ((vel_bits & 0xF)<<4)|(kp_bits>>8)
            data4 = int(kp_bits & 0xFF)
            kd_val = max(min(kd_val, self.max_kd), self.min_kd)
            # kd_bits = int((kd_val-self.min_kd)*self.max_kd_bits/(self.max_kd-self.min_kd))
            kd_bits = float_to_uint(kd_val, self.min_kd, self.max_kd, self.max_kd_bits)
            data5 = kd_bits >> 4
            # !!! This is actually the information about the current
            tau_des = max(min(tau_des, self.max_tau), self.min_tau)
            # tau_bits = int((tau_des-self.min_tau)*self.max_tau_bits/(self.max_tau-self.min_tau))
            tau_bits = float_to_uint(tau_des, self.min_tau, self.max_tau, self.max_tau_bits)
            data6 = ((kd_bits & 0xF)<<4)| (tau_bits >> 8)
            data7 = tau_bits & 0xFF
            cmd.data = [data0, data1, data2, data3, data4, data5, data6, data7]
            # Publish the data to CAN
            self.can_command.publish(cmd)

    def can_msgrecv(self, data):
        # rospy.loginfo('in can msg recv')
        info = data.data
        # Check if the message recieved is for the corresponding motorID
        if info[0] == self.motorID:
            msg = tmotor_data()
            # Now we need to parse the data from the motor to interpretable data
            # the first (info[0]) is reserved for the motorID (1)
            # 1nd and 2rd are upper and lower bits of pos
            pos_bits = float(info[1]<<8|info[2])
            # from bits to rad
            # pos_rad = ((self.max_pos-self.min_pos)/(self.max_pos_bits))*pos_bits+self.min_pos
            pos_rad = uint_to_float(pos_bits, self.min_pos, self.max_pos, self.max_pos_bits)
            # publish position in degree for readability
            msg.position = math.degrees(pos_rad)

            # 3 and 4 data contain 8 bits and 4 bits resp.
            vel_bits = float(info[3]<<4|info[4]>>4)
            # vel_rads = ((self.max_vel-self.min_vel)/(self.max_vel_bits))*vel_bits+self.min_vel
            vel_rads = uint_to_float(vel_bits, self.min_vel, self.max_vel, self.max_vel_bits)
            msg.velocity = vel_rads

            # 4 and 5 have 4 and 8 bits respectively
            # Toruque cannot be observed by passive movement though?
            # !!! Need to check it in running
            tau_bits = float((info[4] & 0x0F)<<8|info[5])
            # tau_nm = (((self.max_tau-self.min_tau)/(self.max_tau_bits))*tau_bits+self.min_tau)*self.Kt
            tau_nm = uint_to_float(tau_bits, self.min_tau, self.max_tau, self.max_tau_bits)*self.Kt*self.gear_ratio
            msg.torque = tau_nm

            self.can_response.publish(msg)
        pass

    # We need to constantly publish something to get the info from the motor
    # def ping_motor(self, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]):
    def ping_motor(self):
        msg = Frame()
        msg.id = self.motorID
        # Since this frame is just to request the data
        # we set the rtr bit to True --> Just a remote frame
        msg.is_rtr = True
        msg.is_extended = False
        msg.is_error = False
        msg.dlc = 0
        self.can_command.publish(msg)

    def disarmMotor(self):
        msg = Frame()
        msg.id = self.motorID
        msg.is_rtr = False
        msg.is_extended = False
        msg.is_error = False
        msg.dlc = 8
        msg.data = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD]
        self.can_command.publish(msg)
        # Set the status of active
        self.motor_active = False

    # # listener
    # def listener(self):
        

if __name__ == '__main__':
    print('motor_testbench node started')
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    # Initiate the class
    mtb = motor_testbench()
    # Set rate
    rate = rospy.Rate(50) # 10hz
    # Then start the loop
    while not rospy.is_shutdown():
        # mtb.ping_motor()
        rate.sleep()
