import numpy as np
from utils.abstractRobotHal import RobotHAL

class Solo12(RobotHAL):
    ''' Define the hardware interface to solo12'''

    def __init__(self, interfaceName="", dt=0.001):
        RobotHAL.__init__(self, interfaceName, dt)

    def InitRobotSpecificParameters(self):
        ''' Definition of the Solo12 paramters '''
        self.nb_motors = 12
        self.motorToUrdf = [0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10]
        self.gearRatio = np.array(self.nb_motors * [9., ])  # gearbox ratio
        self.motorKt = np.array(self.nb_motors * [0.025, ])  # Nm/A
        self.motorSign = np.array([+1, -1, -1, -1, +1, +1,
                                   +1, -1, -1, -1, +1, +1])
        self.maximumCurrent = 3.0  # A
        # To get this offsets, run the calibration with self.encoderOffsets at 0,
        # then manualy move the robot in zero config, and paste the position here (note the negative sign!)
        self.encoderOffsets = - np.array([1.940310, -2.658198, -2.893262, -1.918342,
                                          1.501880, -0.694378, 2.974716, -0.415520, -0.326077, 0.444250, 2.433491, -0.384136])
        # self.encoderOffsets *= 0.
        # 180 degree roll
        self.rotateImuVectors = lambda x: [x[0], -x[1], -x[2]]
        self.rotateImuOrientation = lambda q: [q[3], -q[2], -q[1], q[0]]
