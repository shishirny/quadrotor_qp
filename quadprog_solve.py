#!/usr/bin/env python
""" This code is for solving QP s for our robots """
__author__ = "Shishir Kolathaya"
__status__ = "Testing"

import quadprog
import numpy

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -numpy.vstack([A, G]).T
        qp_b = -numpy.hstack([b, h])
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]

if __name__ == '__main__':
    A = 2
    b = 2
    P = 2
    q = 0

    quadprog_solve_qp(P=P,q=q,A=A,b=b)
