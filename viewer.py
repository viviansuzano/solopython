# coding: utf8

import argparse
import math
from time import clock, sleep
from masterboard_utils import *
from abstractRobotHal import RobotHAL

import pinocchio as pin
import numpy as np
from IPython import embed
import time


pin.switchToNumpyMatrix()
modelPath = "/opt/openrobots/lib/python3.5/site-packages/../../../share/example-robot-data/robots"
urdf = modelPath + "/solo_description/robots/solo.urdf"
robot = pin.RobotWrapper.BuildFromURDF(urdf, modelPath, pin.JointModelFreeFlyer())
robot.initDisplay(loadModel=True)
if ('viewer' in robot.viz.__dict__):
    robot.viewer.gui.setRefreshIsSynchronous(False)


time.sleep(0.2)
q_viewer = robot.q0.copy()

name_interface = "enp4s2f1"
device = RobotHAL(name_interface, dt=0.001, calibrateEncoders=True, logSize=60*1000)


Kp = 0.1
Kd = 0.0
q_ref = np.array([1,-1,1,-1,-1,1,-1,1])


# CONTROL LOOP ***************************************************
while ((not device.hardware.IsTimeout()) and  device.cpt < device.logSize):
    device.UpdateMeasurment()
    device.SetDesiredJointTorque(Kp*(q_ref-device.q_mes))
    device.SendCommand(WaitEndOfCycle=True)

    if ((device.cpt % 50) == 0):
        device.Print()
        quat = device.baseOrientation
        q_viewer[3:7] = np.matrix(quat).T  # IMU Attitude
        q_viewer[7:] = np.matrix(device.q_mes).T  # Encoders
        robot.display(q_viewer)

embed()
# ****************************************************************
# Shut down the interface between the computer and the master board
device.hardware.Stop()
