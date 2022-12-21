# DIVA-1.1.3-V-motor-testbench
Technische Universität München

Laboratory for Product Development and Lightweight Design

Hiwi job: 01.11.2022 - 31.12.2022

Supervisor: Akhil Sathuluri

## Introduction

This is a software infrastructure for testing the DIVA robot using AK-series motors with an inbuild MIT Mini-Cheetah controller. 

We use ROS noetic as the control interface and socket CAN as communication media. A GUI has been established with the PyQt tool to realize the motor connection, feedback monitoring, and command transmission. Several rostopics (e.g. /motor_command and /motor_state) are also available to set the states of the motors and transfer position, velocity, and torque command to the motor. A rostopic called can_response is also ready to read the feedback from the embedded encoder of the motor through the CAN bus.

## Environment

Software:
- ROS noetic
- Python 3.8.10
- Socket CAN

Hardware:
- Tmotor AK series motor, e.g. AK60, AK80, AK10
- One 3D-printed testbench to hold the AK series motor

## Current Progress

- low level message sending and receiving through CAN bus 

pack & unpack --done
    
- low level state command sending 

enable, disable, setZero --done

- low level control command sending 

position, velocity, torque, Kp, Kd --done

- trajectory generation and tracking (for position control) 

linear interpolation and cubic interpolation --done

- position, velocity and torque control accuracy check (with external sensors)

position control check - error is a sine wave in 360 degrees

velocity control check - slightly lower than the set velocity

## Work in the future

- measure backlash and improve the control accuracy
- Solve the problem of the gravitational drag (affects the control accuracy)
- implement teach-playback feature
- RL learning

## Contribution
- Akhil Sathuluri (Project supervisor)
- LPL mechanical team (DIVA platform construction)
- Chi Zhang (Developer)

