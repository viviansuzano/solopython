# coding: utf8

import argparse
import math
from time import perf_counter, sleep
from utils.slider_box import SliderBox
from solo12 import Solo12

def example_script(name_interface):
    slider_box = SliderBox()
    device = Solo12(name_interface,dt=0.001)
    nb_motors = device.nb_motors

    device.init(calibrateEncoders=True)
    #CONTROL LOOP ***************************************************
    start_time = perf_counter()
    while ((not device.hardware.IsTimeout()) and (perf_counter() - start_time< 200) and not slider_box.get_e_stop()):
        device.UpdateMeasurment()
        device.SetDesiredJointTorque([0]*nb_motors)
        device.SendCommand(WaitEndOfCycle=True)
        if ((device.cpt % 100) == 0):
            device.Print()
    #****************************************************************
    
    # Whatever happened we send 0 torques to the motors.
    device.SetDesiredJointTorque([0]*nb_motors)
    device.SendCommand(WaitEndOfCycle=True)

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
