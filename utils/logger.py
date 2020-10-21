'''This class will log 1d array in Nd matrix from device and qualisys object'''
import numpy as np
from datetime import datetime as datetime


class Logger():
    def __init__(self, device, qualisys=None, logSize=60e3, ringBuffer=False):
        self.ringBuffer = ringBuffer
        logSize = np.int(logSize)
        self.logSize = logSize
        self.i = 0
        nb_motors = device.nb_motors

        # Allocate the data:
        # IMU and actuators:
        self.q_mes = np.zeros([logSize, nb_motors])
        self.v_mes = np.zeros([logSize, nb_motors])
        self.torquesFromCurrentMeasurment = np.zeros([logSize, nb_motors])
        self.baseOrientation = np.zeros([logSize, 4])
        self.baseAngularVelocity = np.zeros([logSize, 3])
        self.baseLinearAcceleration = np.zeros([logSize, 3])
        self.baseAccelerometer = np.zeros([logSize, 3])

        # Motion capture:
        self.mocapPosition = np.zeros([logSize, 3])
        self.mocapVelocity = np.zeros([logSize, 3])
        self.mocapAngularVelocity = np.zeros([logSize, 3])
        self.mocapOrientationMat9 = np.zeros([logSize, 3, 3])
        self.mocapOrientationQuat = np.zeros([logSize, 4])

        # Estimator:
        self.estimatorHeight = np.zeros([logSize, 1])
        self.estimatorVelocity = np.zeros([logSize, 3])
        self.contactStatus = np.zeros([logSize, 4])
        self.referenceVelocity = np.zeros([logSize, 6])
        self.logXFMPC = np.zeros([logSize, 24])

    def sample(self, device, qualisys=None, estimator=None):
        if (self.i >= self.logSize):
            if self.ringBuffer:
                self.i = 0
            else:
                return
        # Logging from the device
        self.q_mes[self.i] = device.q_mes
        self.v_mes[self.i] = device.v_mes
        self.baseOrientation[self.i] = device.baseOrientation
        self.baseAngularVelocity[self.i] = device.baseAngularVelocity
        self.baseLinearAcceleration[self.i] = device.baseLinearAcceleration
        self.baseAccelerometer[self.i] = device.baseAccelerometer
        self.torquesFromCurrentMeasurment[self.i] = device.torquesFromCurrentMeasurment
        # Logging from qualisys
        if qualisys is not None:
            self.mocapPosition[self.i] = qualisys.getPosition()
            self.mocapVelocity[self.i] = qualisys.getVelocity()
            self.mocapAngularVelocity[self.i] = qualisys.getAngularVelocity()
            self.mocapOrientationMat9[self.i] = qualisys.getOrientationMat9()
            self.mocapOrientationQuat[self.i] = qualisys.getOrientationQuat()
        # Logging from the estimator
        if estimator is not None:
            self.estimatorHeight[self.i] = estimator.FK_h
            self.estimatorVelocity[self.i] = estimator.v_filt[0:3, 0]
            self.contactStatus[self.i] = estimator.contactStatus
            self.referenceVelocity[self.i] = estimator.v_ref.ravel()
            self.logXFMPC[self.i] = estimator.x_f_mpc.ravel()
        self.i += 1

    def saveAll(self, fileName="data"):
        date_str = datetime.now().strftime('_%Y_%m_%d_%H_%M')

        np.savez(fileName + date_str + ".npz",
                 q_mes=self.q_mes,
                 v_mes=self.v_mes,
                 torquesFromCurrentMeasurment=self.torquesFromCurrentMeasurment,
                 baseOrientation=self.baseOrientation,
                 baseAngularVelocity=self.baseAngularVelocity,
                 baseLinearAcceleration=self.baseLinearAcceleration,
                 baseAccelerometer=self.baseAccelerometer,
                 mocapPosition=self.mocapPosition,
                 mocapVelocity=self.mocapVelocity,
                 mocapAngularVelocity=self.mocapAngularVelocity,
                 mocapOrientationMat9=self.mocapOrientationMat9,
                 mocapOrientationQuat=self.mocapOrientationQuat,
                 estimatorHeight=self.estimatorHeight,
                 estimatorVelocity=self.estimatorVelocity,
                 contactStatus=self.contactStatus,
                 referenceVelocity=self.referenceVelocity,
                 logXFMPC=self.logXFMPC)
