'''This class will log 1d array in Nd matrix from device and qualisys object'''
import numpy as np
class Logger():
    def __init__(self, device, qualisys=None, logSize=60e3, ringBuffer=False):    
        self.ringBuffer = ringBuffer
        self.logSize=logSize
        self.i=0
        nb_motors = device.nb_motors
        
        #allocate the data:
        self.q_mes = np.zeros([logSize,nb_motors])
        self.v_mes = np.zeros([logSize,nb_motors])
        self.baseOrientation = np.zeros([logSize,4])
        self.baseAngularVelocity = np.zeros([logSize,3])
        self.baseLinearAcceleration = np.zeros([logSize,3])

        self.mocapPosition = np.zeros([logSize,3])
        self.mocapVelocity = np.zeros([logSize,3])
        self.mocapOrientationMat9 = np.zeros([logSize,3,3])
        self.mocapOrientationQuat = np.zeros([logSize,4])

    def sample(self,device,qualisys=None):
        if (self.i>=self.logSize):
            if self.ringBuffer:
                self.i=0
            else:
                return
        #Logging from the device
        self.q_mes[self.i] = device.q_mes
        self.v_mes[self.i] = device.v_mes
        self.baseOrientation[self.i] = device.baseOrientation
        self.baseAngularVelocity[self.i] = device.baseAngularVelocity
        self.baseLinearAcceleration[self.i] = device.baseLinearAcceleration

        #Logging from qualisys
        if qualisys is not None:
            self.mocapPosition[self.i] = qualisys.getPosition()
            self.mocapVelocity[self.i] = qualisys.getVelocity()
            self.mocapOrientationMat9[self.i] = qualisys.getOrientationMat9()
            self.mocapOrientationQuat[self.i] = qualisys.getOrientationQuat()
        self.i+=1
        
