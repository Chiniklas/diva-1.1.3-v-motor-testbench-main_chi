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

def float_to_uint(number,v_min,v_max,bits):
    number = number/(v_max-v_min)*2**bits
    if (v_min != 0):
        number = int(number + 2**bits/2)
    else:
        number = int(number)
    return number
# -------------------------------


import rospy
from can_msgs.msg import Frame
import math
import json


max_pos = 12.5 # rad
min_pos = -12.5 # rad
max_pos_bits = 0xFFFF # 16 bits for pos
# velocity !!!  !!! DIFFERENT FROM AMM
max_vel = 45.0 # rad/s
min_vel = -45.0 # rad/s
max_vel_bits = 0xFFF # 12 bits for vel
# torque
max_tau = 15.0
min_tau = -15.0
max_tau_bits = 0xFFF # 12 bits for current
Kt = 0.068 # Nm/A
# For control params
min_kp = 0
max_kp = 500
max_kp_bits = 0xFFF

min_kd = 0
max_kd = 5
max_kd_bits = 0xFFF

def motor_command_callback():
    # Parse the requested motor command and send it as a CAN frame
    # Check if the motor is armed
    p_des = math.radians(30)
    v_des = 0
    tau_des = 0
    kp_val = 200
    kd_val = 2.5
    # We just need to do the reverse scaling done when the data was obtained
    pos_bits = int((p_des-   min_pos)*   max_pos_bits/(   max_pos-   min_pos))
    data0 = pos_bits >> 8
    data1 = pos_bits & 0xFF 
    # velocity float --> bits
    vel_bits = int((v_des- min_vel)*   max_vel_bits/(   max_vel-   min_vel))
    data2 = vel_bits >> 4
    kp_bits = int((kp_val-   min_kp)*   max_kp_bits/(   max_kp-   min_kp))
    data3 = ((vel_bits & 0xF)<<4)|(kp_bits>>8)
    data4 = int(kp_bits & 0xFF)
    kd_bits = int((kd_val-   min_kd)*   max_kd_bits/(   max_kd-   min_kd))
    data5 = kd_bits >> 4
    tau_bits = int((tau_des-   min_tau)*   max_tau_bits/(   max_tau-   min_tau))
    data6 = ((kd_bits & 0xF)<<4)| (tau_bits >> 8)
    data7 = tau_bits & 0xFF
    # cmd.data = [data0, data1, data2, data3, data4, data5, data6, data7]
    bits = [data0, data1, data2, data3, data4, data5, data6, data7]
    print(bits)
    with open("./Sat.json", "w") as fp:
        json.dump(bits, fp)
        print("Done writing JSON data into .json file")


    # Amms calculations
    p_des = float_to_uint(float(p_des),-12.5,12.5,16)
    v_des = float_to_uint(float(v_des),-45,45,12)
    t_des = float_to_uint(float(tau_des),-15,15,12)
    kp_des = float_to_uint(float(kp_val),0,500,12)
    kd_des = float_to_uint(float(kd_val),0,5,12)
    data0 = p_des >>8
    data1 = p_des & 0xFF
    data2 = v_des >>4
    data3 = ((v_des & 0xF)<<4)|(kp_des >>8)
    data4 = kp_des & 0xFF
    data5 = kd_des >> 4
    data6 = ((kd_des & 0xF)<<4)|(t_des >> 8)
    data7 = t_des & 0xFF
    bits = [data0, data1, data2, data3, data4, data5, data6, data7]
    print(bits)
    with open("./Am.json", "w") as fp:
        json.dump(bits, fp)
        print("Done writing JSON data into .json file")



motor_command_callback()