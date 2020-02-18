from multiprocessing import Process, Lock
from multiprocessing.sharedctypes import Value, Array
from ctypes import c_double
import pinocchio as pin
import numpy as np
import time
class NonBlockingViewerFromRobot():
    def __init__(self,robot,dt=0.01):
        # a shared c_double array
        self.dt = dt
        self.shared_q_viewer = Array(c_double, robot.nq, lock=False)
        p = Process(target=self.display_process, args=(robot, self.shared_q_viewer))
        p.start()
        
    def display_process(self,robot, shared_q_viewer):
        ''' This will run on a different process'''
        q_viewer = robot.q0.copy()
        while(1):
            for i in range(robot.nq):
                q_viewer[i] = shared_q_viewer[i]
            robot.display(q_viewer)
            time.sleep(self.dt)

    def display(self,q):
        for i in range(len(self.shared_q_viewer)):
            self.shared_q_viewer[i] = q[i]


class viewerClient():
    def __init__(self,urdf="",modelPath= "/opt/openrobots/lib/python3.5/site-packages/../../../share/example-robot-data/robots",dt=0.01):
        pin.switchToNumpyMatrix()
        urdf = modelPath + "/solo_description/robots/solo.urdf"
        robot = pin.RobotWrapper.BuildFromURDF( urdf, modelPath, pin.JointModelFreeFlyer())
        robot.initDisplay(loadModel=True)   
        if ('viewer' in robot.viz.__dict__):
            robot.viewer.gui.setRefreshIsSynchronous(False)        
        self.nbv = NonBlockingViewerFromRobot(robot,dt)

    def display(self,q):
        self.nbv.display(q)


"""v=viewerClient()
from IPython import embed
embed()"""