#!/usr/bin/env python3

import rospy
import can
import struct
import signal
import time
import sys
import random

from std_msgs.msg import String,Float32,Float64
from can_msgs.msg import Frame
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Point
import numpy as np
from tf.transformations import quaternion_about_axis
from t_motor_api.msg import *
import math

rospy.init_node('mover', anonymous=True)
pub3 = rospy.Publisher('/t_motor/1/', tmotor_cmd, queue_size=10)
pub4 = rospy.Publisher('/t_motor/2/', tmotor_cmd, queue_size=10)
pub5 = rospy.Publisher('/t_motor/3/', tmotor_cmd, queue_size=10)
pub6 = rospy.Publisher('/t_motor/4/', tmotor_cmd, queue_size=10)
pub7 = rospy.Publisher('/t_motor/5/', tmotor_cmd, queue_size=10)
rate = rospy.Rate(50) # 10hz

def exit_gracefully(signum, frame):
	signal.signal(signal.SIGINT, original_sigint)

	sys.exit(1)

	signal.signal(signal.SIGINT, exit_gracefully)

def move1():
    zielzeit = 2 # in millisekunden
    zielwert = math.pi /2
    steigung = zielwert/zielzeit
    start = rospy.get_time() #rospy.Time.now().nsecs/1000
    #print(steigung)
    #while (rospy.Time.now().nsecs/1000<(start + zielzeit)):
    while (rospy.get_time()<(start + zielzeit)):
        #print("hi")
        #position = (rospy.Time.now().nsecs/1000-start) * steigung
        position = (rospy.get_time()-start)*steigung
        positionCMD(1,position)
        positionCMD(2,position)
        positionCMD(3,position)
        positionCMD(4,position)
        positionCMD(5,position)
        positionCMD(6,position)
        positionCMD(7,position)
        rate.sleep()
    #print("Fertig")

def move2():
    zielzeit = 2 # in millisekunden
    zielwert = math.pi /2
    steigung = zielwert/zielzeit
    position = zielwert
    start = rospy.get_time() #rospy.Time.now().nsecs/1000
    #print(steigung)
    #while (rospy.Time.now().nsecs/1000<(start + zielzeit)):
    while (rospy.get_time()<(start + zielzeit)):
        #print("hi")
        #position = (rospy.Time.now().nsecs/1000-start) * steigung
        # position = (rospy.get_time()-start)*steigung
        positionCMD(1,position)
        positionCMD(2,position)
        positionCMD(3,position)
        positionCMD(4,position)
        positionCMD(5,position)
        positionCMD(6,position)
        positionCMD(7,position)
        rate.sleep()
    #print("Fertig")

def move3():
    zielzeit = 2 # in millisekunden
    zielwert = math.pi /2
    steigung = zielwert/zielzeit
    start = rospy.get_time() #rospy.Time.now().nsecs/1000
    #print(steigung)
    #while (rospy.Time.now().nsecs/1000<(start + zielzeit)):
    while (rospy.get_time()<(start + zielzeit)):
        #print("hi")
        #position = (rospy.Time.now().nsecs/1000-start) * steigung
        position = (rospy.get_time()-start)*-steigung +zielwert
        positionCMD(1,position)
        positionCMD(2,position)
        positionCMD(3,position)
        positionCMD(4,position)
        positionCMD(5,position)
        positionCMD(6,position)
        positionCMD(7,position)
        rate.sleep()
    #print("Fertig")

def move4():
    zielzeit = 2 # in millisekunden
    zielwert = math.pi /2
    steigung = zielwert/zielzeit
    position = 0
    start = rospy.get_time() #rospy.Time.now().nsecs/1000
    #print(steigung)
    #while (rospy.Time.now().nsecs/1000<(start + zielzeit)):
    while (rospy.get_time()<(start + zielzeit)):
        #print("hi")
        #position = (rospy.Time.now().nsecs/1000-start) * steigung
        #position = (rospy.get_time()-start)*-steigung +zielwert
        positionCMD(1,position)
        positionCMD(2,position)
        positionCMD(3,position)
        positionCMD(4,position)
        positionCMD(5,position)
        positionCMD(6,position)
        positionCMD(7,position)
        rate.sleep()
    #print("Fertig")

def positionCMD(motor, position):
    if motor>2:
        st = tmotor_cmd()
        st.position = position
        st.kp = 200
        st.kd = math.sqrt(7)
        st.status = True
        st.setzero = False
        st.velocity = 0
        st.torque = 0
        #print(position)
        if motor==3:
            pub3.publish(st);
        # if motor==4:
        #     pub4.publish(st);
        # if motor==5:
        #     pub5.publish(st);
        # if motor==6:
        #     pub6.publish(st);
        # if motor==7:
        #     pub7.publish(st);

def on_clickStop_Connect():
    st = tmotor_cmd()
    st.position = 0
    st.kp = 0
    st.kd = 0
    st.status = False
    st.setzero = False
    st.velocity = 0
    st.torque = 0
    pub3.publish(st)
    pub4.publish(st)
    pub5.publish(st)
    pub6.publish(st)
    pub7.publish(st)

def on_clickStart_Connect():
    st = tmotor_cmd()
    st.position = 0
    st.kp = 0
    st.kd = 0
    st.status = True
    st.setzero = False
    st.velocity = 0
    st.torque = 0
    pub3.publish(st)
    pub4.publish(st)
    pub5.publish(st)
    pub6.publish(st)
    pub7.publish(st)

def on_clicksetZero():
    st = tmotor_cmd()
    st.position = 0
    st.kp = 0
    st.kd = 0
    st.status = False
    st.setzero = True
    st.velocity = 0
    st.torque = 0
    pub3.publish(st)
    pub4.publish(st)
    pub5.publish(st)
    pub6.publish(st)
    pub7.publish(st)

if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    try:
        #on_clickStop_Connect()
        #on_clicksetZero()
        on_clicksetZero()
        on_clicksetZero()
        on_clicksetZero()
        on_clickStart_Connect()
        for i in range (3):
            move1()
            move2()
            move3()
            move4()
        on_clickStop_Connect()
    except rospy.ROSInterruptException:
        pass
