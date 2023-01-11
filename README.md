# DIVA-1.1.3-V-motor-testbench
Technische Universität München

Laboratory for Product Development and Lightweight Design

Hiwi job: 01.11.2022 - 15.01.2023

Supervisor: Akhil Sathuluri

## Introduction

This is a software infrastructure for testing the DIVA robot using AK-series motors with an inbuild MIT Mini-Cheetah controller. 

We use ROS noetic as the control interface and socket CAN as communication media. A GUI has been established with the PyQt tool to realize the motor connection, feedback monitoring, and command transmission. Several rostopics (e.g. /motor_command and /motor_state) are also available to set the states of the motors and transfer position, velocity, and torque command to the motor. A rostopic called can_response is also ready to read the feedback from the embedded encoder of the motor through the CAN bus.

A 3D-printed testbench for AK60 has been build to enable position, velocity and torque measurement. Extensions for dummy external sensors are made possible.

![96a18d4371097de3a39c24e1d93f4a3](https://user-images.githubusercontent.com/92475185/208930091-be37e68b-dff0-48b7-9391-5dd91e9d114c.png)

![461cda189bff3e5d5bf7098b15d19fb](https://user-images.githubusercontent.com/92475185/208930174-4dfce609-5faf-437a-b336-bd9ec2d5edb9.jpg)


## Environment

Software:
- ROS noetic
- Python 3.8.10
- Socket CAN

Hardware:
- Tmotor AK series motor, e.g. AK60, AK80, AK10
- One 3D-printed testbench to hold the AK series motor
- 24V power supply
- CAN cable

## Current Progress

- low level message sending and receiving through CAN bus 

    - pack & unpack --done
    
- low level state command sending 

    - enable, disable, setZero --done

- low level control command sending 

    - position, velocity, torque, Kp, Kd --done

- trajectory generation and tracking (for position control) 

    - linear interpolation and cubic interpolation --done

- position, velocity and torque control accuracy check (with external sensors)

    - position control check - error is a sine wave in 360 degrees

    - velocity control check - slightly lower than the set velocity

## Work in the future

- measure backlash and improve the control accuracy
- Solve the problem of the gravitational drag (affects the control accuracy)
- implement teach-playback feature
- RL learning

## Documentation
- from the manufacturer:
    - AK60-6: https://store.tmotor.com/goods.php?id=1138
    - AK80-9: https://store.tmotor.com/goods.php?id=982
    - AK10-9: https://store.tmotor.com/goods.php?id=1188

- useful videos:
    - https://www.youtube.com/watch?v=hbqQCgebaF8&list=LL&index=7
    - https://www.youtube.com/watch?v=HzY9vzgPZkA&list=LL&index=20
    - https://www.youtube.com/watch?v=WKRLlthr9kY&list=LL&index=11
 
## Contribution
- Akhil Sathuluri (Project supervisor)
    - GUI design
    - CAN communication setup
    - low level position & velocity control
    - trajectory generation & tracking
    
- LPL mechanical team (DIVA platform construction)

- Chi Zhang (Developer)
    - low level torque control
    - code testing
    - testbench CAD design
    - position, velocity and torque control accuracy check (with external sensors)

