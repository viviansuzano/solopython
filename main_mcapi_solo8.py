# coding: utf8
import numpy as np
import argparse
import math
from time import clock, sleep
from solo8 import Solo8
from pynput import keyboard

import matplotlib.pyplot as plt
from math import ceil
import curves
from multicontact_api import ContactSequence
curves.switchToNumpyArray()
DT = 0.001
KP = 4.
KD = 0.05
KT = 1.
tau_max = 3. * np.ones(8)

key_pressed = False



def config_12_to_8(q12):
	"""
	Remove the shoulder roll joint from the vector
	"""
	assert q12.shape[0] == 12
	return np.hstack((q12[1:3], q12[4:6], q12[7:9], q12[10:12]))


def compute_pd(q_desired, v_desired, tau_desired, device):
	pos_error = q_desired - device.q_mes
	vel_error = v_desired - device.v_mes
	tau = KP * pos_error + KD * vel_error + KT * tau_desired
	tau = np.maximum(np.minimum(tau, tau_max), -tau_max) 
	return tau

def on_press(key):
	global key_pressed
	try:
		if key == keyboard.Key.enter:
			key_pressed = True
			# Stop listener
			return False
	except AttributeError:
		print('Unknown key {0} pressed'.format(key))

def put_on_the_floor(device, q_init):
	global key_pressed
	key_pressed = False
	Kp_pos = 3.
	Kd_pos = 0.01
	imax = 3.0
	pos = np.zeros(device.nb_motors)
	for motor in range(device.nb_motors):
		pos[motor] = q_init[device.motorToUrdf[motor]] * device.gearRatioSigned[motor]
	listener = keyboard.Listener(on_press=on_press)
	listener.start()
	print("Put the robot on the floor and press Enter")
	while not key_pressed:
		device.UpdateMeasurment()
		for motor in range(device.nb_motors):
			ref = Kp_pos*(pos[motor] - device.hardware.GetMotor(motor).GetPosition() - Kd_pos*device.hardware.GetMotor(motor).GetVelocity())
			ref = min(imax, max(-imax, ref))
			device.hardware.GetMotor(motor).SetCurrentReference(ref)
		device.SendCommand(WaitEndOfCycle=True)

	print("Start the motion.")

def plot_mes_des_curve(times, mes_t, des_t = None, title = None, y_label = None):
	colors =  ['r', 'b']
	labels = ["Shoulder" , "Knee"]
	legs_order = ["front left", "front right", "hind left", "hind right"]
	# joint positions des/mes
	fig, ax = plt.subplots(2, 2)
	fig.canvas.set_window_title(title)
	fig.suptitle(title, fontsize=20)
	for i in range(2): # line (front/back)
		for j in range(2):  # column (left/right)
			ax_sub = ax[i, j]
			for k in range(2): # joint
				ax_sub.plot(times, mes_t[i*4 + j*2 + k, :].T, color=colors[k], label=labels[k])
				if des_t is not None:
					ax_sub.plot(times, des_t[i*4 + j*2 + k, :].T, color=colors[k], label=labels[k] + "(ref)", linestyle="dashed")
			ax_sub.set_xlabel('time (s)')
			ax_sub.set_ylabel(y_label)
			ax_sub.set_title(legs_order[i*2 + j])
			ax_sub.legend()



def mcapi_playback(name_interface, filename):
	device = Solo8(name_interface,dt=DT)
	nb_motors = device.nb_motors

	q_viewer = np.array((7 + nb_motors) * [0.,])

	# Load contactsequence from file:
	cs = ContactSequence(0)
	cs.loadFromBinary(filename)
	# extract (q, dq, tau) trajectories:
	q_t = cs.concatenateQtrajectories()  # with the freeflyer configuration
	dq_t = cs.concatenateDQtrajectories()  # with the freeflyer configuration
	ddq_t = cs.concatenateDDQtrajectories()  # with the freeflyer configuration
	tau_t = cs.concatenateTauTrajectories()  # joints torques
	# Get time interval from planning:
	t_min = q_t.min()
	t_max = q_t.max()
	print("## Complete duration of the motion loaded: ", t_max - t_min)
	# Sanity checks:
	assert t_min < t_max
	assert dq_t.min() == t_min
	assert ddq_t.min() == t_min
	assert tau_t.min() == t_min
	assert dq_t.max() == t_max
	assert ddq_t.max() == t_max
	assert tau_t.max() == t_max
	assert q_t.dim() == 19
	assert dq_t.dim() == 18
	assert ddq_t.dim() == 18
	assert tau_t.dim() == 12

	num_steps = ceil((t_max - t_min) / DT) + 1
	q_mes_t = np.zeros([8, num_steps])
	v_mes_t = np.zeros([8, num_steps])
	q_des_t = np.zeros([8, num_steps])
	v_des_t = np.zeros([8, num_steps])
	tau_des_t = np.zeros([8, num_steps])
	tau_send_t = np.zeros([8, num_steps])
	tau_mesured_t = np.zeros([8, num_steps])
	q_init = config_12_to_8(q_t(t_min)[7:])
	device.Init(calibrateEncoders=True, q_init=q_init)
	t = t_min
	put_on_the_floor(device, q_init)
	#CONTROL LOOP ***************************************************
	t_id = 0
	while ((not device.hardware.IsTimeout()) and (t < t_max)):
		q_desired = config_12_to_8(q_t(t)[7:])  # remove freeflyer
		dq_desired = config_12_to_8(dq_t(t)[6:])  # remove freeflyer
		tau_desired = config_12_to_8(tau_t(t))
		device.UpdateMeasurment()
		tau = compute_pd(q_desired, dq_desired, tau_desired, device)
		device.SetDesiredJointTorque(tau)

		# store desired and mesured data for plotting
		q_des_t[:, t_id] = q_desired
		v_des_t[:, t_id] = dq_desired
		q_mes_t[:, t_id] = device.q_mes
		v_mes_t[:, t_id] = device.v_mes
		tau_des_t[:, t_id] = tau_desired
		tau_send_t[:, t_id] = tau
		tau_mesured_t[: , t_id] = device.torquesFromCurrentMeasurment

		device.SendCommand(WaitEndOfCycle=True)
		if ((device.cpt % 100) == 0):
		    device.Print()

		q_viewer[3:7] = device.baseOrientation  # IMU Attitude
		q_viewer[7:] = device.q_mes  # Encoders
		t += DT
		t_id += 1
	#****************************************************************
    
	# Whatever happened we send 0 torques to the motors.
	device.SetDesiredJointTorque([0]*nb_motors)
	device.SendCommand(WaitEndOfCycle=True)

	if device.hardware.IsTimeout():
		print("Masterboard timeout detected.")
		print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
	device.hardware.Stop()  # Shut down the interface between the computer and the master board
	
	# Plot the results
	times = np.arange(t_min, t_max + DT, DT)
	plot_mes_des_curve(times, q_mes_t, q_des_t, "Joints positions", "joint position")
	plot_mes_des_curve(times, v_mes_t, v_des_t, "Joints velocities", "joint velocity")
	plot_mes_des_curve(times, tau_send_t, tau_des_t, "Joints torque", "Nm")
	current_t = np.zeros([8, num_steps])
	for motor in range(device.nb_motors):
		current_t[device.motorToUrdf[motor], :] = tau_send_t[device.motorToUrdf[motor], :] / device.jointKtSigned[motor]
	plot_mes_des_curve(times, current_t, title="Motor current", y_label="A")

	tracking_pos_error = q_des_t - q_mes_t
	plot_mes_des_curve(times, tracking_pos_error, title="Tracking error")

	plot_mes_des_curve(times, tau_mesured_t, title="Torque mesured from current", y_label="nM")
	current_mesured_t = np.zeros([8, num_steps])
	for motor in range(device.nb_motors):
		current_mesured_t[device.motorToUrdf[motor], :] = tau_mesured_t[device.motorToUrdf[motor], :] / device.jointKtSigned[motor]
	plot_mes_des_curve(times, current_mesured_t, title="Measured motor current", y_label="A")

	plt.show()

def main():
    parser = argparse.ArgumentParser(description='Playback trajectory stored in a multicontact-api file.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    parser.add_argument('-f',
						'--filename',
						type=str,
						required=True,
                        help="The absolute path of a multicontact_api.ContactSequence serialized file")

    mcapi_playback(parser.parse_args().interface, parser.parse_args().filename)


if __name__ == "__main__":
    main()
