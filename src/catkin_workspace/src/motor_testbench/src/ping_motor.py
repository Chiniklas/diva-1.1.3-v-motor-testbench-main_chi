#!/usr/bin/env python3


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

rospy.init_node('motor_pinger')
can_command = rospy.Publisher('sent_messages', Frame, queue_size = 10)

# We need to constantly publish something to get the info from the motor
# def ping_motor(self, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]):
def ping_motor():
    msg = Frame()
    msg.id = 1
    # Since this frame is just to request the data
    # we set the rtr bit to True --> Just a remote frame
    msg.is_rtr = True
    msg.is_extended = False
    msg.is_error = False
    msg.dlc = 8
    can_command.publish(msg)

if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    # Initiate the class
    # Set rate
    rate = rospy.Rate(50) # 10hz
    # Then start the loop
    while not rospy.is_shutdown():
        ping_motor()
        rate.sleep()
