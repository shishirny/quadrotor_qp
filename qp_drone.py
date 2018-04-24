
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
    kp = 80.
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

    mean_rpm = 310

    servomin = 160
    servomax = 524
    i1 = 1
    b = 0
    b0 = 0
    kd = 20

    t0 = time.clock()
    T = 0
    dt = 0.001

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
    while 1:

        before=time.clock()
	time_total =  time.clock() - t_start
        if imu.IMURead():
            # x, y, z = imu.getFusionData()
            # print("%f %f %f" % (x,y,z))
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
	    fusionVel = data["gyro"]
            #print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
            time.sleep(poll_interval*1.0/1000.0)

	    roll_offset = 2.*3.14/180.

            u_roll=qp_q_dot_des(fusionPose[0] + roll_offset) - kd * fusionVel[0]
 #           print int(u_roll[0])
 #           rpm_in_front = 340 + int(u_roll[0])
 #           rpm_in_back = 340 + int(-u_roll[0])

            u_pitch=qp_q_dot_des(fusionPose[1]) - kd * fusionVel[1]
#            print int(u_pitch[0])
#            rpm_in_left = 340 + int(u_pitch[0])
#            rpm_in_right = 340 + int(-u_pitch[0])


	    u_0 = mean_rpm - int(u_roll) - int(u_pitch)
	    u_1 = mean_rpm + int(u_roll) + int(u_pitch)
	    u_2 = mean_rpm + int(u_roll) - int(u_pitch)
	    u_3 = mean_rpm - int(u_roll) + int(u_pitch)

#	    print [fusionPose[0], fusionPose[1]]

#	    print [do_saturate(rpm_in_front), do_saturate(rpm_in_back), do_saturate(rpm_in_left), do_saturate(rpm_in_right)]

#        after=time.clock()
#        print after-before
    	    pwm1.set_pwm(0, 0, do_saturate(u_0))
            pwm1.set_pwm(1, 0, do_saturate(u_1))
	    pwm1.set_pwm(2, 0, do_saturate(u_2))
	    pwm1.set_pwm(3, 0, do_saturate(u_3))
            time.sleep(0.0001)
