# Typing-Robot
Simple self made typing robot using Raspberry Pi

## Overview

The goal of this project is to build a robotic arm that can type on a keyboard automatically by using robotics control systems. The structure of the robot is a simple robotic arm with 3 Degree of Freedoms (DOFS) to move around over a generic keyboard. All start with a piece of wood. So first let's build the basic model of our typign robot!

## Solidworks Model

We used Solidworks to design the #D model of our robot, shown in the following figure. Our robot is primarily constructed from 3 parts: a rail, link 1 and link 2, which constitute the 3 DOFs.

![](https://github.com/Zhi29/Zhi29.github.io/tree/master/images/projects/SWmodel.png)

## Electrical Component and Design

The electrical components used to build the system include: an L293D curcuit, a Raspberry Pi, Hall Effect Sensors, Power Supply and Geared DC Motor. The L293D is an H bridge motor control circuit which has a high current output; the Raspberry Pi is the central procesor where the control loop runs. The hall effect sensors detect changes in magnetic fields so a pair of them can function as an encoder using a phase offset between the pair to infer direction of rotation. The power supply provides 6 volts at a current appropriate for the motors and the motors are the outputof the sustem and have a high gear ratio which allowed adequate torque generation.

![](https://github.com/Zhi29/Zhi29.github.io/tree/master/images/projects/electricalcomponent.jpg)

This figure shows the connections between the components. 

## Control Methods

Since the arm of the robot moves against gracity after each key stroke, the effect of gravity is non negligible. Thus the PD Conrtol with gravity compensation was used to control the robot. To see the detail of the control algorithms, please refer the pdf in my github repository of this project.

The following is the real shape of our typing robot.
![](https://github.com/Zhi29/Zhi29.github.io/tree/master/images/projects/real.png)

The result of typing robot is shown in the snapshots
![](https://github.com/Zhi29/Zhi29.github.io/tree/master/images/projects/result.jpg)

