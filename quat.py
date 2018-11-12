#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Christian Bolander
MAE 6510
11.1-11.3
"""

from math import sin, cos, asin, atan2, pi


def QuatMult(a, b):
    a0 = a[0]
    a1 = a[1]
    a2 = a[2]
    a3 = a[3]
    b0 = b[0]
    b1 = b[1]
    b2 = b[2]
    b3 = b[3]
    return [a0*b0 - a1*b1 - a2*b2 - a3*b3,
            a0*b1 + a1*b0 + a2*b3 - a3*b2,
            a0*b2 - a1*b3 + a2*b0 + a3*b1,
            a0*b3 + a1*b2 - a2*b1 + a3*b0]


def Euler2Quat(euler):
    C0 = cos(euler[0]/2.0)
    C1 = cos(euler[1]/2.0)
    C2 = cos(euler[2]/2.0)
    S0 = sin(euler[0]/2.0)
    S1 = sin(euler[1]/2.0)
    S2 = sin(euler[2]/2.0)
    return [C0*C1*C2 + S0*S1*S2,
            S0*C1*C2 - C0*S1*S2,
            C0*S1*C2 + S0*C1*S2,
            C0*C1*S2 - S0*S1*C2]


def Quat2Euler(q):
    e0 = q[0]
    e1 = q[1]
    e2 = q[2]
    e3 = q[3]
    condition = (e0*e2 - e1*e3)
    # Returns phi, theta, psi
    if condition == 0.5:
        return [2.0*asin(e1/cos(pi/4.0)),
                pi/2.0,
                0.0]
    elif condition == -0.5:
        return [2.0*asin(e1/cos(pi/4.0)),
                -pi/2.0,
                0.0]
    else:
        return [atan2(2.0*(e0*e1 + e2*e3),
                      (e0**2 + e3**2 - e1**2 - e2**2)),
                asin(2.0*condition),
                atan2(2.0*(e0*e3 + e1*e2),
                      (e0**2 + e1**2 - e2**2 - e3**2))]


def Body2Fixed(v_b, q):
    q_mat = [[q[1]**2 - q[2]**2 - q[3]**2,
              2.0*(q[1]*q[2]),
              2.0*(q[1]*q[3])],
             [2.0*(q[1]*q[2]),
              q[2]**2 - q[1]**2 - q[3]**2,
              2.0*(q[2]*q[3])],
             [2.0*(q[1]*q[3]),
              2.0*(q[3]*q[2]),
              q[3]**2 - q[1]**2 - q[2]**2]]
    return [q_mat[0][0]*v_b[0] + q_mat[0][1]*v_b[1] + q_mat[0][2]*v_b[2],
            q_mat[1][0]*v_b[0] + q_mat[1][1]*v_b[1] + q_mat[1][2]*v_b[2],
            q_mat[2][0]*v_b[0] + q_mat[2][1]*v_b[1] + q_mat[2][2]*v_b[2]]


def Fixed2Body(v_f, q):
    q_mat = [[q[1]**2 + q[0]**2 - q[2]**2 - q[3]**2,
              2.0*(q[1]*q[2] + q[3]*q[0]),
              2.0*(q[1]*q[3] - q[2]*q[0])],
             [2.0*(q[1]*q[2] - q[3]*q[0]),
              q[2]**2 + q[0]**2 - q[1]**2 - q[3]**2,
              2.0*(q[2]*q[3] + q[1]*q[0])],
             [2.0*(q[1]*q[3] + q[2]*q[0]),
              2.0*(q[3]*q[2] - q[1]*q[0]),
              q[3]**2 + q[0]**2 - q[1]**2 - q[2]**2]]
    return [q_mat[0][0]*v_f[0] + q_mat[0][1]*v_f[1] + q_mat[0][2]*v_f[2],
            q_mat[1][0]*v_f[0] + q_mat[1][1]*v_f[1] + q_mat[1][2]*v_f[2],
            q_mat[2][0]*v_f[0] + q_mat[2][1]*v_f[1] + q_mat[2][2]*v_f[2]]


def NormalizeQuaternion(q):
    return [x*(1.5 - 0.5*(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)) for x in q]
