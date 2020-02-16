from multiprocessing import Process, Lock
from multiprocessing.sharedctypes import Value, Array
from ctypes import c_double
import pinocchio as pin
import numpy as np
import time
class NonBlockingViewer():
    def __init__(self,robot,timeIntervals=0.05):
        # a shared c_double array
        self.timeIntervals = timeIntervals
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
            time.sleep(self.timeIntervals)

    def display(self,q):
        for i in range(len(self.shared_q_viewer)):
            self.shared_q_viewer[i] = q[i]


def benchmark():
    pin.switchToNumpyMatrix()
    modelPath = "/opt/openrobots/lib/python3.5/site-packages/../../../share/example-robot-data/robots"
    urdf = modelPath + "/solo_description/robots/solo.urdf"
    robot = pin.RobotWrapper.BuildFromURDF( urdf, modelPath, pin.JointModelFreeFlyer())
    robot.initDisplay(loadModel=True)   

    def job(n,t,disp):
        for i in range(n):
            disp(robot.q0)
            robot.q0[7:]+=0.01
            time.sleep(t)

    #blocking, in sync
    t=time.time()
    job(10000,0.01,robot.display)
    print (time.time()-t)

    #blocking, not synchronous
    robot.viewer.gui.setRefreshIsSynchronous(False)
    t=time.time()
    job(10000,0.01,robot.display)
    print (time.time()-t)

    #non blocking
    nbv = NonBlockingViewer(robot,0.0)
    t=time.time()
    job(10000,0.01,nbv.display)
    print (time.time()-t)