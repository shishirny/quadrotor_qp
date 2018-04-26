#!/usr/bin/python
# coding: utf-8

from __future__ import division

import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
from smbus import SMBus
import Adafruit_PCA9685
import subprocess
import math
from math import pi
import psutil
import quadprog
import time
import socket
import numpy as np
import struct

#from .quadrotor_qp.quadprog_solve import qp_q_dot_des
#import os
from numpy import array

# Global Variables
pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
#pwm2 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
pwm1.set_pwm_freq(50)
#pwm2.set_pwm_freq(50)
bus = SMBus(1)

p = psutil.Process(os.getpid())

p.nice(-19)
p.cpu_affinity([3])


def quadprog_solve_qp(H, h, A=None, b=None, C=None, d=None):
    qp_H = .5 * (H + H.T)   # make sure H is symmetric
    qp_h = -h
    if C is not None:
        qp_C = -numpy.vstack([C, A]).T
        qp_d = -numpy.hstack([d, b])
        meq = C.shape[0]
    else:  # no equality constraint
        qp_C = -A.T
        qp_d = -b
        meq = 0

    # print qp_H
    # print qp_h
    # print qp_C
    # print qp_d
    return quadprog.solve_qp(qp_H, qp_h, qp_C, qp_d, meq)[0]

def qp_q_dot_des(q_act):
    q_des = 0.

    H = array([[100.,0.],[0.,1.]])  # cost function matrix is given here   e.g. u^T H u
    h = array([0.,0.])  # cost function vector    e.g. h^T u

    # stability constraints
    kp = 40.
    Va = q_act - q_des
    Vb = -kp * (q_act - q_des)*(q_act - q_des)

    ## safety constraints
    angle_limit = 25. # in radians - very high
    Ba = - 2. * q_act # derivative of angle_limit - x^2
    Bb = -50. * (angle_limit - q_act*q_act) # - (angle_limit - x^2)

    A = array([[-1, Va],[0,-Ba]])  # inequality constraints are given here Au \leq b
    b = array([Vb,-Bb])

    u_in = quadprog_solve_qp(H, h, A, b)

    return array([u_in[1]])

def do_saturate(input):
    if input > 420:
	input = 420
    elif input < 227:
       	input = 227
    return input


if __name__ == '__main__':

    SETTINGS_FILE = "RTIMULib"

    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    print("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded")

    # this is a good time to set any fusion parameters

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)

    # Functions
    #pwm1 = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
    #imudevice = Adafruit_PCA9685.PCA9685(address=0x68, busnum=1)

    mean_rpm_0 = 300
    mean_rpm_1 = 300
    mean_rpm_2 = 290
    mean_rpm_3 = 310

#    servomin = 227
#    servomax = 420
#    i1 = 1
#    b = 0
#    b0 = 0

    ki_0 = 3.
    kp_0 = 20.
    kd_0 = 6.

    ki_1 = 3.
    kp_1 = 20.
    kd_1 = 6.

    ki_2 = 3.
    kp_2 = 20.
    kd_2 = 6.

    ki_3 = 3.
    kp_3 = 20.
    kd_3 = 6.

    t0 = time.clock()
    T = 0
    dt = 0.003

    # here the quadprog is run    # These must be the inputs
    #q_des = 0. # desired angle
    q_act = 2. # actual angle -- this is the input from the sensor
    #q_dot_des = 0.  # this is desired velocity
    #q_dot_act = 3.  # this is actual velocity

    t_start = time.clock()
    pwm1.set_pwm(0, 0, 227)
    pwm1.set_pwm(1, 0, 227)
    pwm1.set_pwm(2, 0, 227)
    pwm1.set_pwm(3, 0, 227)
    time.sleep(5)

    fusionInt= array([0.,0.])
    diff = 0
    before = 0
    while 1:
        if imu.IMURead():
#	    before=time.clock()
            # x, y, z = imu.getFusionData()
            # print("%f %f %f" % (x,y,z))
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
	    fusionVel = data["gyro"]
	    fusionInt[0] = fusionInt[0] + dt*fusionPose[0]
            fusionInt[1] = fusionInt[1] + dt*fusionPose[1]
            #print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
            time.sleep(poll_interval*1.0/1000.0)

	    roll_offset = 2.*3.14/180.

            u_roll_0= - kp_0 * (fusionPose[0] + roll_offset) - kd_0 * fusionVel[0] - ki_0 * fusionInt[0]
            u_roll_1= - kp_1 * (fusionPose[0] + roll_offset) - kd_1 * fusionVel[0] - ki_1 * fusionInt[0]
            u_roll_2= - kp_2 * (fusionPose[0] + roll_offset) - kd_2 * fusionVel[0] - ki_2 * fusionInt[0]
            u_roll_3= - kp_3 * (fusionPose[0] + roll_offset) - kd_3 * fusionVel[0] - ki_3 * fusionInt[0]
#            u_roll=qp_q_dot_des(fusionPose[0] + roll_offset) - kd * fusionVel[0]
 #           print int(u_roll[0])

            u_pitch_0= - kp_0 * fusionPose[1] - kd_0 * fusionVel[1] - ki_0 * fusionInt[1]
            u_pitch_1= - kp_1 * fusionPose[1] - kd_1 * fusionVel[1] - ki_1 * fusionInt[1]
            u_pitch_2= - kp_2 * fusionPose[1] - kd_2 * fusionVel[1] - ki_2 * fusionInt[1]
            u_pitch_3= - kp_3 * fusionPose[1] - kd_3 * fusionVel[1] - ki_3 * fusionInt[1]
#            u_pitch=qp_q_dot_des(fusionPose[1]) - kd * fusionVel[1]
#            print int(u_pitch[0])


	    u_0 = mean_rpm_0 - int(u_roll_0) - int(u_pitch_0)
	    u_1 = mean_rpm_1 + int(u_roll_1) + int(u_pitch_1)
	    u_2 = mean_rpm_2 + int(u_roll_2) - int(u_pitch_2)
	    u_3 = mean_rpm_3 - int(u_roll_3) + int(u_pitch_3)



    	    pwm1.set_pwm(0, 0, do_saturate(u_0))
            pwm1.set_pwm(1, 0, do_saturate(u_1))
	    pwm1.set_pwm(2, 0, do_saturate(u_2))
	    pwm1.set_pwm(3, 0, do_saturate(u_3))
            time.sleep(0.0001)

	    message = ""
	    message += struct.pack(">f",u_0)
	    message += struct.pack(">f",u_1)
            soc = socket.socket()
   	    host = ''  # ip of computer
	    port = 12345
	    soc.connect((host, port))
	    soc.send(message)
	    soc.close()

            after=time.clock()
            diff  = after - before
	    before = time.clock()
