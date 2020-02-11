# coding: utf8

import argparse
import math
from time import clock, sleep
from masterboard_utils import *
from abstractRobotHal import RobotHAL

def example_script(name_interface):
    device = RobotHAL(name_interface,dt=0.001,calibrateEncoders=True)

    #CONTROL LOOP ***************************************************
    while ((not device.hardware.IsTimeout()) and (clock() < 200)): 
        device.UpdateMeasurment()
        device.SetDesiredJointTorque([0]*8)
        device.SendCommand(WaitEndOfCycle=True) 
        if ((device.cpt % 100) == 0):
            device.Print()
    #****************************************************************

    if device.hardware.IsTimeout():
        print("Masterboard timeout detected.")
        print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
    device.hardware.Stop()  # Shut down the interface between the computer and the master board

def main():
    parser = argparse.ArgumentParser(description='Example masterboard use in python.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    example_script(parser.parse_args().interface)


if __name__ == "__main__":
    main()
