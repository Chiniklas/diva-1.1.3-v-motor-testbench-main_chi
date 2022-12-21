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


## Current Progress

- [ ] [Set up project integrations](https://gitlab.lrz.de/lpl-tum/diva/diva-1/diva-1.1.3-v/diva-1.1.3-v-motor-testbench/-/settings/integrations)

## Work in the future

- measure backlash and improve the control accuracy
- Solve the problem of the gravitational drag (affects the control accuracy)
- implement teach-playback function
- RL learning

## Contribution
- Akhil Sathuluri (Project supervisor)
- LPL mechanical team (DIVA platform construction)
- Chi Zhang (Developer)

