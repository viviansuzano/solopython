""" 
This client connects to a Quaisys server with an asyncronous subprocess and expose 6d position and velocity of a given body
Thomas FLAYOLS - LAAS CNRS
"""

import asyncio
from multiprocessing import Process, Lock
from multiprocessing.sharedctypes import Value, Array
from ctypes import c_double
import qtm
import numpy as np
class QualisysClient():
    def __init__(self,ip="127.0.0.1",body_id=0):
        #shared c_double array
        self.shared_bodyPosition = Array(c_double, 3, lock=False)
        self.shared_bodyVelocity = Array(c_double, 3, lock=False)
        self.shared_bodyOrientationQuat = Array(c_double, 4, lock=False)
        self.shared_bodyOrientationMat9 = Array(c_double, 9, lock=False)
        self.shared_timestamp = Value(c_double, lock=False)
        #self.shared_timestamp = -1
        args=(ip, body_id, self.shared_bodyPosition, self.shared_bodyVelocity,self.shared_bodyOrientationQuat,self.shared_bodyOrientationMat9, self.shared_timestamp)
        self.p = Process(target=self.qualisys_process, args=args)
        self.p.start()

    def stop(self):
        self.p.terminate()
        self.p.join()
    
    def getPosition(self):
        return np.array([self.shared_bodyPosition[0],
                         self.shared_bodyPosition[1],
                         self.shared_bodyPosition[2]])
    def getVelocity(self):
        return np.array([self.shared_bodyVelocity[0],
                         self.shared_bodyVelocity[1],
                         self.shared_bodyVelocity[2]])

    def getOrientationMat9(self):
        return np.array([[self.shared_bodyOrientationMat9[0],self.shared_bodyOrientationMat9[1],self.shared_bodyOrientationMat9[2]],
                         [self.shared_bodyOrientationMat9[3],self.shared_bodyOrientationMat9[4],self.shared_bodyOrientationMat9[5]],
                         [self.shared_bodyOrientationMat9[6],self.shared_bodyOrientationMat9[7],self.shared_bodyOrientationMat9[8]]])

    def getOrientationQuat(self):
        return np.array([self.shared_bodyOrientationQuat[0],
                         self.shared_bodyOrientationQuat[1],
                         self.shared_bodyOrientationQuat[2],
                         self.shared_bodyOrientationQuat[3]])
        pass

    def qualisys_process(self, ip,body_id, shared_bodyPosition,shared_bodyVelocity,shared_bodyOrientationQuat,shared_bodyOrientationMat9,shared_timestamp):
        print("Qualisys process!")
        ''' This will run on a different process'''
        shared_timestamp.value = -1
        def on_packet(packet):
                """ Callback function that is called everytime a data packet arrives from QTM """
                position=packet.get_6d()[1][body_id][0]
                orientation=packet.get_6d()[1][body_id][1]
                timestamp = packet.timestamp * 1e-6

                position_x = position.x * 1e-3
                position_y = position.y * 1e-3
                position_z = position.z * 1e-3

                #Compute world velocity
                if 0:#(shared_timestamp.value == -1):
                    shared_bodyVelocity[0] = 0 
                    shared_bodyVelocity[1] = 0 
                    shared_bodyVelocity[2] = 0 
                else:
                    dt = timestamp - shared_timestamp.value
                    shared_bodyVelocity[0] = (position_x-shared_bodyPosition[0])/dt
                    shared_bodyVelocity[1] = (position_y-shared_bodyPosition[1])/dt
                    shared_bodyVelocity[2] = (position_z-shared_bodyPosition[2])/dt
                
                shared_bodyPosition[0] = position_x
                shared_bodyPosition[1] = position_y
                shared_bodyPosition[2] = position_z
                
                shared_bodyOrientationQuat[0] = 0#TODO
                shared_bodyOrientationQuat[1] = 0#TODO
                shared_bodyOrientationQuat[2] = 0#TODO
                shared_bodyOrientationQuat[3] = 1#TODO

                for i in range(9):
                    shared_bodyOrientationMat9[i]=orientation.matrix[i]

                last_position_x = position.x
                last_position_y = position.y
                last_position_z = position.z

                shared_timestamp.value = timestamp
        async def setup():
            """ Main function """
            connection = await qtm.connect(ip)
            if connection is None:
                print("no connection with qualisys!")
                return
            print("Connected")
            try:
                await connection.stream_frames(components=["6d"], on_packet=on_packet)
            except:
                print("connection with qualisys lost")

        asyncio.ensure_future(setup())
        asyncio.get_event_loop().run_forever()

def exampleOfUse():
    import time
    qc = QualisysClient(ip="140.93.16.160",body_id=7)
    for i in range(20):
        print(qc.getPosition())
        time.sleep(0.3)
    print("killme!")

#exampleOfUse()