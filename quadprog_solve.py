import time
import serial
import subprocess
ser = serial.Serial('/dev/ttyACM0')

ser.baudrate = 9600
ser.isOpen()

#!/usr/bin/env python
""" This code is for solving QP s for our robots """
__author__ = "Shishir Kolathaya"
__status__ = "Testing"

import quadprog
import numpy as np
from numpy import array, dot
import serial

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

def simple_pd(q_act,q_dot_act):
    q_des = 0.
    u_in = 5*(q_des - q_act)
    return u_in

def qp_q_dot_des(q_act):
    q_des = 0.

    H = array([[100.,0.],[0.,1.]])  # cost function matrix is given here   e.g. u^T H u
    h = array([0.,0.])  # cost function vector    e.g. h^T u

    # stability constraints
    kp = 100.
    Va = q_act - q_des
    Vb = -kp * (q_act - q_des)*(q_act - q_des)

    ## safety constraints
    angle_limit = 2. # in radians
    Ba = - 2. * q_act # derivative of angle_limit - x^2
    Bb = - (angle_limit - q_act*q_act) # - (angle_limit - x^2)

    # all inequality constraints are included here
    #A = array([[(q_act - q_des)]]) # inequality constraints are given here Au \leq b
    #b = array([-100. * (q_act - q_des)*(q_act - q_des)])

    A = array([[-1, Va],[0,-Ba]])  # inequality constraints are given here Au \leq b
    b = array([Vb,-Bb])

    u_in = quadprog_solve_qp(H, h, A, b)

    return u_in

if __name__ == '__main__':
    # These must be the inputs
    q_des = 0. # desired angle
    q_act = 2. # actual angle -- this is the input from the sensor
    q_dot_des = 0.  # this is desired velocity
    q_dot_act = 3.  # this is actual velocity

    #P = [[1.732, 1],[1,1,732]]
    #F = [[0, 1],[0, 0]]
    #G = [[0],[1]]
    #V = (x_act - x_des)*P*(x_act - x_des)

    # mass =  3.  # this is the mass of the lever arm in kilograms
    '''k_p = 10.  # proportional gain
    k_d = 1.   # derivative gain

    H = array([[1.]]) # cost function matrix is given here   e.g. u^T H u
    h = array([0.]) # cost function vector    e.g. h^T u

    A = array([[2.*(q_dot_act - q_dot_des)]])
    b = array([-2.*(q_act - q_des)*k_p*q_dot_act - q_dot_act*k_d*q_dot_act])

    result = quadprog_solve_qp(H, h, A, b)'''
    u_in=0
    while 1:
        #subprocess.call(["tail", "-n","1","/dev/ttyACM0"])
        #subprocess.call(["sed","-n","2p","/dev/ttyACM0"])
        q_act=float(ser.readline())
        #u_in=simple_pd(q_act,0.)
        u_in=qp_q_dot_des(q_act)
	#u_in = 12
	ser.write("a");
	if u_in<10 and u_in>-10:
            ser.write(str(0))
        ser.write(str(int(u_in[1])))
        #ser.write("-15")
        #ser.write("\n")
	#print str(int(u_in))
        #ser.write(str(10))
        print u_in[1];
        #time.sleep(1)
        #time.sleep(0.01)
