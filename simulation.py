#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 25 12:35:45 2018

@author: christian
"""

import numpy as np
import quat
import json
import stdatmos as atmos
import classicalrungekutta as RK


def load_file(filename, a_obj):
    f = json.load(open(filename))
    a_obj.const_den = f['simulation']['constant_density']
    dt = f['simulation']['timestep']
    a_obj.S_w = f['aircraft']['wing_area']
    a_obj.b_w = f['aircraft']['wing_span']
    a_obj.z_T_o = f['aircraft']['thrust']['offset']
    a_obj.T0 = f['aircraft']['thrust']['T0']
    a_obj.T1 = f['aircraft']['thrust']['T1']
    a_obj.T2 = f['aircraft']['thrust']['T2']
    a_obj.a = f['aircraft']['thrust']['a']
    a_obj.V_o = f['initial']['airspeed']
    a_obj.latitude = f['initial']['latitude']
    a_obj.longitude = f['initial']['longitude']
    a_obj.altitude = f['initial']['altitude']
    a_obj.W = f['initial']['weight']
    a_obj.climb = np.deg2rad(f['initial']['climb'])
    a_obj.bank = np.deg2rad(f['initial']['bank'])
    a_obj.heading = np.deg2rad(f['initial']['heading'])
    a_obj.V_ref = f['reference']['airspeed']
    a_obj.rho_ref = f['reference']['density']
    a_obj.de_ref = np.deg2rad(f['reference']['elevator'])
    a_obj.CL_ref = f['reference']['lift']
    a_obj.CD_ref = f['reference']['CD']
    a_obj.Ixx = f['reference']['Ixx']
    a_obj.Iyy = f['reference']['Iyy']
    a_obj.Izz = f['reference']['Izz']
    a_obj.Ixy = f['reference']['Ixy']
    a_obj.Ixz = f['reference']['Ixz']
    a_obj.Iyz = f['reference']['Iyz']
    a_obj.CL_a = f['reference']['CL,a']
    a_obj.CD_a = f['reference']['CD,a']
    a_obj.CD_a_a = f['reference']['CD,a,a']
    a_obj.Cm_a = f['reference']['Cm,a']
    a_obj.CY_b = f['reference']['CY,b']
    a_obj.Cll_b = f['reference']['Cl,b']
    a_obj.Cn_b = f['reference']['Cn,b']
    a_obj.CL_q = f['reference']['CL,q']
    a_obj.CD_q = f['reference']['CD,q']
    a_obj.Cm_q = f['reference']['Cm,q']
    a_obj.CY_p = f['reference']['CY,p']
    a_obj.Cll_p = f['reference']['Cl,p']
    a_obj.Cn_p = f['reference']['Cn,p']
    a_obj.CY_r = f['reference']['CY,r']
    a_obj.Cll_r = f['reference']['Cl,r']
    a_obj.Cn_r = f['reference']['Cn,r']
    a_obj.CL_de = f['reference']['CL,de']
    a_obj.CD_de = f['reference']['CD,de']
    a_obj.Cm_de = f['reference']['Cm,de']
    a_obj.CY_da = f['reference']['CY,da']
    a_obj.Cll_da = f['reference']['Cl,da']
    a_obj.Cn_da = f['reference']['Cn,da']
    a_obj.CY_dr = f['reference']['CY,dr']
    a_obj.Cll_dr = f['reference']['Cl,dr']
    a_obj.Cn_dr = f['reference']['Cn,dr']
    a_obj.C_D3 = f['reference']['CD3']
    return dt


def initialize(a_obj):
    a_obj.rho = atmos.statee(a_obj.altitude)[-1]
    a_obj.rho_0 = atmos.statee(0)[-1]
    a_obj.state_vars[8] = -a_obj.altitude
    a_obj.norm_const_trim = 0.5*a_obj.rho*(a_obj.V_o**2)*a_obj.S_w
    q_ref = 0.5*a_obj.rho_ref*(a_obj.V_ref**2)*a_obj.S_w
    a_obj.c_w = a_obj.S_w/a_obj.b_w
    a_obj.I_inv = np.linalg.inv([[a_obj.Ixx, -a_obj.Ixy, -a_obj.Ixz],
                                 [-a_obj.Ixy, a_obj.Iyy, -a_obj.Iyz],
                                 [-a_obj.Ixz, -a_obj.Iyz, a_obj.Izz]])
    a_obj.CL_ref = a_obj.CL_ref/q_ref
    a_obj.trim()
    a_obj.state_vars[-4:] = quat.Euler2Quat([a_obj.bank, a_obj.elevation,
                                             a_obj.heading])
    a_obj.state_vars[-4:] = quat.NormalizeQuaternion(a_obj.state_vars[-4:])
    a_obj.state_vars[0] = a_obj.u_o
    a_obj.state_vars[1] = a_obj.v_o
    a_obj.state_vars[2] = a_obj.w_o
    a_obj.state_vars[3] = a_obj.p_o
    a_obj.state_vars[4] = a_obj.q_o
    a_obj.state_vars[5] = a_obj.r_o


def run_sim(a_obj, dt):
    states_p1 = RK.classical_RK4(0.0, a_obj.state_vars, dt,
                                 a_obj.eq_o_st)
    states_p1[-4:] = quat.NormalizeQuaternion(states_p1[-4:])
    a_obj.geographic_coords(states_p1[6:9])
    a_obj.state_vars = np.copy(states_p1)
    a_obj.V_now = np.sqrt(a_obj.state_vars[0]**2 + a_obj.state_vars[1]**2 +
                          a_obj.state_vars[2]**2)
    a_obj.alpha_now = np.arctan2(a_obj.state_vars[2], a_obj.state_vars[0])
    a_obj.beta_now = np.arctan2(a_obj.state_vars[1], a_obj.state_vars[0])