#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 16:10:15 2018

@author: christian
"""
import numpy as np
import quat
import sinesummationpredictivefit as sspf


class Aircraft:
    """This Aircraft class defines the geometry and inherent properties of an
    aircraft to be used in simulation.

    Attributes
    ------------
    filename : string
        A file name containing information on the physical attributes of the
        aircraft.
    state_vars : array_like
        The state variables for the aircraft (u, v, w, p, q, r, x, y, z, e0,
        ex, ey, ez) at a given point in time.
    .
    .
    .

    See Also
    -----------
    quat.py : Quaternion manipulation module.

    json : Used for extracting information from a .json file.

    stdatmos.py : Gives atmospheric conditions in SI or English units for a
    given altitude.

    classicalrungekutta.py : Implements a 4th-order Runge Kutta method of
    integration to be used in estimating future time-steps of `state_vars`.

    """
    def __init__(self):
        # Geometry information (Given)
        self.S_w = 0.0  # Planform area of the wing
        self.b_w = 0.0  # Span of the wing
        self.c_w = 0.0  # Chord of the wing
        self.V_o = 0.0  # Initial airspeed
        self.W = 0.0  # Weight of the aircraft
        self.Ixx = 0.0  # Inertia tensor of the aircraft
        self.Iyy = 0.0
        self.Izz = 0.0
        self.Ixy = 0.0
        self.Ixz = 0.0
        self.Iyz = 0.0
        self.I_inv = np.zeros((3, 3))  # Inverted inertia matrix (Calculated)
        self.l_r = 0.0

        # Aerodynamic Information (Given)
        self.CL_a = 0.0  # Reference Lift Slope
        self.CD_a = 0.0  # Reference dDrag/dalpha
        self.CD_a_a = 0.0  # Reference dDrag/dalpha/dalpha
        self.Cm_a = 0.0  # Reference dPitching Moment/dalpha
        self.CL_ah = 0.0  # Reference dLift/dalpha_hat
        self.CD_ah = 0.0  # Reference dDrag/dalpha_hat
        self.Cm_ah = 0.0  # Reference dPitching Moment/dalpha_hat
        self.CY_b = 0.0  # Reference dSide Force/dbeta
        self.Cll_b = 0.0  # Reference dRolling Moment/dbeta
        self.Cn_b = 0.0  # Reference dYawing Moment/dbeta
        self.CL_q = 0.0  # Reference dLift/droll_rate
        self.CD_q = 0.0  # Reference dDrag/droll_rate
        self.Cm_q = 0.0  # Reference dPitching Moment/droll_rate
        self.CY_p = 0.0  # Reference dSide Force/dpitch_rate
        self.Cll_p = 0.0  # Reference dRolling Moment/dpitch_rate
        self.Cn_p = 0.0  # Reference dYawing Moment/dpitch_rate
        self.CY_r = 0.0  # Reference dSide Force/dyaw_rate
        self.Cll_r = 0.0  # Reference dRolling Moment/dyaw_rate
        self.Cn_r = 0.0  # Reference dYawing Moment/dyaw_rate
        self.CL_de = 0.0  # Reference dLift/delevator
        self.CD_de = 0.0  # Reference dDrag/delevator
        self.Cm_de = 0.0  # Reference dPitching Moment/delevator
        self.CY_da = 0.0  # Reference dSide Force/daileron
        self.Cll_da = 0.0  # Reference dRolling Moment/daileron
        self.Cn_da = 0.0  # Reference dYawing Moment/daileron
        self.CY_dr = 0.0  # Reference dSide Force/drudder
        self.Cll_dr = 0.0  # Reference dRolling Moment/drudder
        self.Cn_dr = 0.0  # Reference dYawing Moment/drudder
        self.F_M = np.zeros(6)  # Aerodynamic forces/moments (Fx->Fz,Mx->Mz)
        self.T_F_M = np.zeros(6)  # Thrust forces/moments

        # Initial orientation
        self.elevation = 0.0  # Initial elevation angle (calculated)
        self.climb = 0.0  # Initial climb angle (given)
        self.bank = 0.0  # Initial bank angle (given)
        self.heading = 0.0

        #  Wind, control surfaces, aircraft configuration, and atmosphere
        self.const_den = True  # Flag for using constant density
        self.M = 0.0  # Mach Number
        self.state_vars = np.zeros(13)  # State variables
        self.controls = np.zeros(4)  # Control in (elevator, aileron, rudder, throttle)
        self.de_ref = 0.0  # Initial elevator deflection (given)
        self.V_wind = np.zeros(3)  # 3 component wind vector
        self.g = 32.2  # Gravity (ft/s^2)
        self.rho = 0.0  # Current calculated density based on altitude
        self.rho_ref = 0.0  # Reference density
        self.rho_0 = 0.0  # Sea level density
        self.hx = 0.0  # Angular momentum generators
        self.hy = 0.0
        self.hz = 0.0
        self.V_ref = 0.0  # Reference velocity
        self.norm_const_trim = 0.0  # calculated 1/2rho(V_o^2)S_w

        # Thrust variables
        self.x_T_o = 0.0  # Axial propulsion offset
        self.y_T_o = 0.0  # Spanwise propulsion offset
        self.z_T_o = 0.0  # Vertical propulsion offset
        self.T0 = 0.0  # Thrust model coefficients
        self.T1 = 0.0
        self.T2 = 0.0
        self.a = 0.0

        # Trim variables
        self.CD_ref = 0.0  # Trim drag coefficient
        self.C_D0 = 0.0  # Drag model coefficient
        self.C_D1 = 0.0  # Drag model coefficient
        self.C_D2 = 0.0  # Drag model coefficient
        self.C_D3 = 0.0  # Drag model coefficient
        self.CL_ref = 0.0  # Lift coefficient at reference state
        self.tau_o = 0.0  # Trim throttle setting
        self.alpha_o = 0.0  # Trim angle of attack
        self.beta_o = 0.0  # Trim sideslip angle
        self.Cm_ref = 0.0  # Reference Pitching Moment (calculated)
        self.de_o = 0.0  # Trim elevator deflection
        self.da_o = 0.0  # Trim aileron deflection
        self.dr_o = 0.0  # Trim rudder deflection
        self.u_o = 0.0  # Trim x-velocity
        self.v_o = 0.0  # Trim y-velocity
        self.w_o = 0.0  # Trim z-velocity
        self.p_o = 0.0  # Trim rolling rate
        self.q_o = 0.0  # Trim pitching rate
        self.r_o = 0.0  # Trim yawing rate

        # Current variables
        self.V_now = 0.0
        self.alpha_now = 0.0
        self.beta_now = 0.0

        # Stall function info
        self.lift_harm = np.array([7.0660907938962945, -0.11627448224464568,
                                   -1.1749114729936403, 0.2674725623246685,
                                   7.5191215216609475, -0.11593888249622532,
                                   8.02973807891213, 0.03082232032852557,
                                   0.43866548007124306, 0.9188178661917805,
                                   0.021238133646525156, 0.49627497341375676,
                                   -4.829946159759168, 0.39543577948446484,
                                   0.13577895133394066, -0.15206031495815736,
                                   0.05744195095106385, 0.2912986652696257,
                                   4.2843464997030365, -0.5739177458517997,
                                   0.020214426209248088, 3.7916363635033603,
                                   1.6972470992459598, 0.10690136987220167,
                                   -0.5474359057155165, -0.06374243309620785,
                                   0.2153757340594828, 0.8358848712464275,
                                   -0.09481802859720262, 0.053790054905202424,
                                   2.31645279576617, -0.2537502216797848,
                                   0.19570770820829136, 3.8595845181869417,
                                   0.20318087023579634, 0.1507718903482557,
                                   -0.8698920598969072, -0.0617174554145617,
                                   0.2181232670655577, 0.636278581806304,
                                   0.008373817956981108, 0.19726471404738902,
                                   2.108230304467757, 0.05259903521887621,
                                   0.1868550777876811, 3.6352915082188186,
                                   0.04827583532227138, 0.1973237399910856,
                                   -1.1182796181883399, 0.010430645042350063,
                                   0.1900113633679816, 0.38642316431139184,
                                   0.03546154086565831, 0.1897861291009869,
                                   1.8827667758421756, 0.03611893788439122,
                                   0.1900437530547133, 3.4197535524948517,
                                   0.025760239590755265, 0.1949345821049455,
                                   -1.3486404388325817, 0.004001958041569592,
                                   0.19016086017759834, 0.15470330220305156,
                                   0.019573580648607425, 0.18410301662785483,
                                   1.629455914405167, 0.022309209989323422,
                                   0.18350632884515636, 3.1813320173090625,
                                   0.016500453021978632, 0.18542351171884533,
                                   -1.564499855827339, 0.003192473864688782,
                                   0.18992754988961794, -0.04598651052784301,
                                   0.007763254629132172, 0.1833381898490255,
                                   1.392335413934736, 0.008896465574220025,
                                   0.18352124530190955, 2.9746277233030995,
                                   0.00776560330931156, 0.18206048973554206,
                                   4.595698150184711, 0.0069492891694282275,
                                   0.18167331837262693, -0.2743318710331607,
                                   0.005846900217210234, 0.18140300938973017,
                                   1.1106188147721414])
        self.drag_harm = np.array([1.0, 0.043, np.pi - 0.1, 1.05])
        self.moment_harm = np.array([52.92082241831228, -0.032105581818538384,
                                     5.91639070974744, 25.929194195431116,
                                     26.687512645465382, 0.04654418631208993,
                                     6.1118507894561445, 0.37039380836080416,
                                     0.44729839464684906, -5.324502306324748,
                                     -0.32559811265212146, 0.4571137063125297,
                                     0.5400816636923348, -0.09736951119396256,
                                     0.32335110070843776, -0.3979366252106608,
                                     -0.04743656475920655, 0.2525902990728076,
                                     -1.5643043113160358, 0.04380736976337287,
                                     0.23034906159685087, 3.1286462992587913,
                                     0.018942867031032762, 0.25383902374700557,
                                     1.5858930918923535, 0.01382029662088026,
                                     0.2650823400310552, -0.21623297906506603,
                                     0.00217321622397017, 0.25831230629554036,
                                     4.434087855207149, 0.0033921812834298757,
                                     0.25374288882227963, 2.9926756007729227,
                                     0.0013591716755926165, 0.3826355133114705,
                                     3.237553222836382, 0.0016557163476443918,
                                     0.25657835723378963, 4.122063652819904,
                                     0.0031177793320597705, 0.3828902381837064,
                                     3.812497232425366])

    def eq_o_st(self, t, y0):
        """Describes the equation of state for the aircraft.

        """
        u = y0[0]
        v = y0[1]
        w = y0[2]
        p = y0[3]
        q = y0[4]
        r = y0[5]
        e0 = y0[9]
        ex = y0[10]
        ey = y0[11]
        ez = y0[12]
        g = self.g
        W = self.W
        Ixx = self.Ixx
        Iyy = self.Iyy
        Izz = self.Izz
        Ixy = self.Ixy
        Ixz = self.Ixz
        Iyz = self.Iyz
        h_terms = np.array([-self.hz*q + self.hy*r,
                            self.hz*p - self.hx*r,
                            -self.hy*p + self.hx*q])
        #  Call aerodynamics to evaluate forces and moments.
        self.aerodynamics(y0)
        udot = (2.0*g*(ex*ez - ey*e0) + (g/W)*(self.F_M[0] + self.T_F_M[0]) +
                (r*v - q*w))
        vdot = (2.0*g*(ey*ez + ex*e0) + (g/W)*(self.F_M[1] + self.T_F_M[1]) +
                (p*w - r*u))
        wdot = (g*(ez**2 + e0**2 - ex**2 - ey**2) +
                (g/W)*(self.F_M[2] + self.T_F_M[2]) +
                (q*u - p*v))
        p_q_r_corr = np.zeros(3)
        p_q_r_corr[0] = (self.F_M[3] + self.T_F_M[3] + (Iyy - Izz)*q*r +
                         Iyz*(q**2 - r**2) + Ixz*p*q - Ixy*p*r)
        p_q_r_corr[1] = (self.F_M[4] + self.T_F_M[4] + (Izz - Ixx)*p*r +
                         Ixz*(r**2 - p**2) + Ixy*q*r - Iyz*p*q)
        p_q_r_corr[2] = (self.F_M[5] + self.T_F_M[5] + (Ixx - Iyy)*p*q +
                         Ixy*(p**2 - q**2) + Iyz*p*r - Ixz*q*r)
        p_q_rdot = np.matmul(self.I_inv, (h_terms + p_q_r_corr))
        xyz = quat.QuatMult([e0, ex, ey, ez], quat.QuatMult([0, u, v, w],
                            [e0, -ex, -ey, -ez]))[1:] + self.V_wind
        e_s = [0.5*((p*-ex) + (q*-ey) + (r*-ez)),
               0.5*((p*e0) + (q*-ez) + (r*ey)),
               0.5*((p*ez) + (q*e0) + (r*-ex)),
               0.5*((p*-ey) + (q*ex) + (r*e0))]
        return np.array([udot,
                         vdot,
                         wdot,
                         p_q_rdot[0],
                         p_q_rdot[1],
                         p_q_rdot[2],
                         xyz[0],
                         xyz[1],
                         xyz[2],
                         e_s[0],
                         e_s[1],
                         e_s[2],
                         e_s[3]
                         ])

    def aerodynamics(self, temp_vars):
        alpha = np.arctan2(temp_vars[2], temp_vars[0])
        beta = np.arctan2(temp_vars[1], temp_vars[0])
        a_control = 20*np.pi/180.
        a_u = 12.0*np.pi/180.
        a_l = 10.0*np.pi/180.
        u = temp_vars[0]
        v = temp_vars[1]
        w = temp_vars[2]
        V = np.sqrt(u**2 + v**2 + w**2)
        pbar = temp_vars[3]*self.b_w/(2.0*V)
        qbar = temp_vars[4]*self.c_w/(2.0*V)
        rbar = temp_vars[5]*self.b_w/(2.0*V)
        de = self.controls[0] - self.de_ref
        da = self.controls[1]
        dr = self.controls[2]
        norm_const = 0.5*self.rho*self.S_w*V**2
        D = self.drag_harm
        dragmod = D[0]*np.cos(D[1]*np.rad2deg(alpha) + D[2]) + D[3]
        CS = (self.CY_b*beta + self.CY_p*pbar + self.CY_r*rbar +
              self.CY_da*da + self.CY_dr*dr)
        CLconst = (self.CL_ref + self.CL_q*qbar + self.CL_de*de)
        CDconst = (self.C_D0 + self.C_D3*(CS**2) + self.CD_q*qbar +
                   self.CD_de*de)
        Cllconst = (self.Cll_b*beta + self.Cll_p*pbar + self.Cll_da*da +
                    self.Cll_dr*dr)
        Cmconst = (self.Cm_ref + self.Cm_q*qbar + self.Cm_de*de)
        Cnconst = (self.Cn_r*rbar + self.Cn_da*da + self.Cn_dr*dr)
        if alpha < a_l:
            CL = CLconst + self.CL_a*alpha
            CD = CDconst + self.C_D1*CL + self.C_D2*(CL**2)
            C_ll = Cllconst + (self.Cll_r/self.CL_ref)*CL*rbar
            Cm = (Cmconst +
                  (self.Cm_a/self.CL_a)*(CL*u/V - self.CL_ref + CD*w/V))
            Cn = ((self.Cn_b/self.CY_b)*(CS*u/V - CD*v/V) +
                  (self.Cn_p/self.CL_ref)*CL*pbar + Cnconst)
        if alpha >= a_l and alpha <= a_u:
            weight = ((alpha - a_u)/(a_l - a_u))
            CL = (CLconst + (1. - weight)*self.CL_a*alpha +
                  weight*sspf.sum_sines(np.rad2deg(alpha), self.lift_harm,
                                        len(self.lift_harm)))
            CD = (CDconst + (1. - weight)*(self.C_D1*CL + self.C_D2*(CL**2)) +
                  weight*(dragmod))
            C_ll = Cllconst + (self.Cll_r/self.CL_ref)*CL*rbar
            Cm = (Cmconst + (1. - weight)*((self.Cm_a/self.CL_a)*(CL*u/V -
                                                                  self.CL_ref +
                                                                  CD*w/V)) +
                  weight*sspf.sum_sines(np.rad2deg(alpha), self.moment_harm,
                                        len(self.moment_harm)))
            Cn = ((self.Cn_b/self.CY_b)*(CS*u/V - CD*v/V) +
                  (self.Cn_p/self.CL_ref)*CL*pbar + Cnconst)
        if alpha > a_u:
            if alpha > a_control:
                CS = (self.CY_b*beta + self.CY_p*pbar + self.CY_r*rbar)
                CLconst = (self.CL_ref + self.CL_q*qbar)
                CDconst = (self.C_D0 + self.C_D3*(CS**2) + self.CD_q*qbar)
                Cllconst = (self.Cll_b*beta + self.Cll_p*pbar)
                Cmconst = (self.Cm_ref + self.Cm_q*qbar)
                Cnconst = (self.Cn_r*rbar)
            if alpha < np.deg2rad(60.):
                CL = CLconst + sspf.sum_sines(np.rad2deg(alpha),
                                              self.lift_harm,
                                              len(self.lift_harm))
                CD = CDconst + dragmod
                C_ll = Cllconst + (self.Cll_r/self.CL_ref)*CL*rbar
                Cm = Cmconst + sspf.sum_sines(np.rad2deg(alpha),
                                              self.moment_harm,
                                              len(self.moment_harm))
                Cn = ((self.Cn_b/self.CY_b)*(CS*u/V - CD*v/V) +
                      (self.Cn_p/self.CL_ref)*CL*pbar + Cnconst)
            if alpha >= np.deg2rad(60.):
                CL = (-0.45/30.)*np.rad2deg(alpha) + 1.3
                if CDconst + dragmod > 2.0:
                    CD = 2.0
                else:
                    CD = CDconst + dragmod
                C_ll = Cllconst + (self.Cll_r/self.CL_ref)*CL*rbar
                Cm = -0.6
                Cn = ((self.Cn_b/self.CY_b)*(CS*u/V - CD*v/V) +
                      (self.Cn_p/self.CL_ref)*CL*pbar + Cnconst)

        print(alpha, CL, CD, Cm)
        self.F_M[0] = (norm_const*(CL*np.sin(alpha) - CS*np.sin(beta) -
                       CD*u/V))  # X aerodynamic force
        self.F_M[1] = (norm_const*(CS*np.cos(beta) -
                       CD*v/V))  # Y aerodynamic force
        self.F_M[2] = (norm_const*(-CL*np.cos(alpha) -
                       CD*w/V))   # Z aerodynamic force
        self.F_M[3] = (norm_const*self.b_w*C_ll)  # Aerodynamic rolling moment
        self.F_M[4] = (norm_const*self.c_w*Cm)  # Aerodynamic pitching moment
        self.F_M[5] = (norm_const*self.b_w*Cn)  # Aerodynamic yawing moment
        self.thrust(temp_vars)

    def mass_change(self, t):
        mass = self.W/self.g*t  # Mass as a function of time
        return mass

    def wind(self, t):
        self.V_wind = np.zeros(3)*t  # Wind as a function of time

    def thrust(self, temp_vars):
        """Describes contribution of thrust to aerodynamic forces

        Attributes
        -------------
        T_F_M : array_like
            Represents the thrust forces and moments in the following order:
            X-thrust, Y-thrust, Z-thrust, X-thrust moment, Y-thrust moment,
            Z-thrust moment.
        """
        V = np.sqrt(temp_vars[0]**2 + temp_vars[1]**2 + temp_vars[2]**2)
        thrust = (self.T0 + self.T1*V + self.T2*(V**2))
        self.T_F_M[0] = (self.controls[3]*(self.rho/self.rho_0)**self.a)*thrust
        self.T_F_M[1] = 0.0
        self.T_F_M[2] = 0.0
        self.T_F_M[3] = self.y_T_o*self.T_F_M[2] - self.z_T_o*self.T_F_M[1]
        self.T_F_M[4] = self.z_T_o*self.T_F_M[0] - self.x_T_o*self.T_F_M[2]
        self.T_F_M[5] = self.x_T_o*self.T_F_M[1] - self.y_T_o*self.T_F_M[0]

    def trim(self):
        Vo = self.V_o
        A = np.zeros((6, 6))
        B = np.zeros(6)
        Sp = np.sin(self.bank)
        Cp = np.cos(self.bank)
        C_W = self.W/(self.norm_const_trim)
        g = self.g
        bw = self.b_w
        cw = self.c_w
        hz = self.hz
        hy = self.hy
        hx = self.hx
        Ixx = self.Ixx
        Iyy = self.Iyy
        Ixy = self.Ixy
        Izz = self.Izz
        Iyz = self.Iyz
        Ixz = self.Ixz
        self.C_D2 = (self.CD_a_a/(2.*self.CL_a**2))
        self.C_D1 = (self.CD_a/self.CL_a) - 2.0*self.C_D2*self.CL_ref
        self.C_D0 = (self.CD_ref - self.C_D1*self.CL_ref -
                     self.C_D2*self.CL_ref**2)
        self.Cm_ref = -(self.z_T_o/cw)*self.CD_ref
        A[3, 2] = bw*self.Cll_b
        A[3, 4] = bw*self.Cll_da
        A[3, 5] = bw*self.Cll_dr
        A[4, 3] = cw*self.Cm_de
        A[5, 4] = bw*self.Cn_da
        A[5, 5] = bw*self.Cn_dr
        alpha_guess = 0.0
        de_guess = self.de_ref
        beta_guess = 0.0
        da_guess = 0.0
        dr_guess = 0.0
        condition = np.array([100.0, 100.0, 100.0, 100.0, 100.0])
        while all(x > 1e-18 for x in condition):
            SA = np.sin(alpha_guess)
            CA = np.cos(alpha_guess)
            SB = np.sin(beta_guess)
            CB = np.cos(beta_guess)
            Vconst = ((Vo)/np.sqrt(1. - (SA**2)*(SB**2)))
            self.u_o = Vconst*(CA*CB)
            self.v_o = Vconst*(CA*SB)
            self.w_o = Vconst*(SA*CB)
            self.climb_to_elevation(alpha_guess, beta_guess)
            St = np.sin(self.elevation)
            Ct = np.cos(self.elevation)
            Omega = (Sp*Ct*g/(St*self.w_o + Cp*Ct*self.u_o))
            self.p_o = Omega*(-St)
            self.q_o = Omega*(Sp*Ct)
            self.r_o = Omega*(Cp*Ct)
            pb_o = (1./(2.*Vo))*bw*self.p_o
            qb_o = (1./(2.*Vo))*cw*self.q_o
            rb_o = (1./(2.*Vo))*bw*self.r_o
            thrust = self.T0 + self.T1*Vo + self.T2*Vo**2
            CT0 = ((self.rho/self.rho_0)**self.a)*(thrust)/self.norm_const_trim
            CL = (self.CL_ref + self.CL_a*alpha_guess + self.CL_q*qb_o +
                  self.CL_de*(de_guess - self.de_ref))
            CS = (self.CY_b*beta_guess + self.CY_p*pb_o + self.CY_r*rb_o +
                  self.CY_da*da_guess + self.CY_dr*dr_guess)
            CD = (self.C_D0 + self.C_D1*CL + self.C_D2*(CL**2) +
                  self.C_D3*(CS**2) + self.CD_q*qb_o +
                  self.CD_de*(de_guess - self.de_ref))
            A[0, 0] = CT0
            A[0, 3] = -self.CD_de*self.u_o/Vo
            A[1, 2] = self.CY_b*CB
            A[1, 4] = self.CY_da*CB
            A[1, 5] = self.CY_dr*CB
            A[2, 1] = -self.CL_a*CA
            A[2, 3] = -self.CL_de*CA
            A[4, 0] = self.z_T_o*CT0
            B[0] = (-CL*SA + CS*SB +
                    (self.C_D0 + self.C_D1*CL + self.C_D2*(CL**2) +
                     self.C_D3*(CS**2) + self.CD_q*qb_o)*self.u_o/Vo +
                    C_W*(St - (self.r_o*self.v_o - self.q_o*self.w_o)/g))
            B[1] = ((-self.CY_p*pb_o - self.CY_r*rb_o)*CB + CD*self.v_o/Vo +
                    C_W*(-Sp*Ct - (self.p_o*self.w_o - self.r_o*self.u_o)/g))
            B[2] = ((self.CL_ref + self.CL_q*qb_o)*CA + CD*self.w_o/Vo +
                    C_W*(-Cp*Ct - (self.q_o*self.u_o - self.p_o*self.v_o)/g))
            B[3] = (-bw*(self.Cll_p*pb_o + (self.Cll_r/self.CL_ref)*CL*rb_o) +
                    (C_W/self.W)*(hz*self.q_o - hy*self.r_o -
                    (Iyy - Izz)*self.q_o*self.r_o -
                    Iyz*(self.q_o**2 - self.r_o**2) -
                    Ixz*self.p_o*self.q_o + Ixy*self.p_o*self.r_o))
            B[4] = (-cw*(self.Cm_ref +
                         (self.Cm_a/self.CL_a)*(CL*self.u_o/Vo -
                                                self.CL_ref +
                                                CD*self.w_o/Vo) +
                         (self.Cm_q*qb_o)) +
                    (C_W/self.W)*(-hz*self.p_o + hx*self.r_o -
                    (Izz - Ixx)*self.p_o*self.r_o -
                    Ixz*(self.r_o**2 - self.p_o**2) -
                    Ixy*self.q_o*self.r_o + Iyz*self.p_o*self.q_o))
            B[5] = (-bw*((self.Cn_b/self.CY_b)*(CS*self.u_o/Vo -
                                                CD*self.v_o/Vo) +
                         (self.Cn_p/self.CL_ref)*CL*pb_o + self.Cn_r*rb_o) +
                    (C_W/self.W)*(hy*self.p_o - hx*self.q_o -
                    (Ixx - Iyy)*self.p_o*self.q_o -
                    Ixy*(self.p_o**2 - self.q_o**2) -
                    Iyz*self.p_o*self.r_o + Ixz*self.q_o*self.r_o))
            ans = np.linalg.solve(A, B)
            alpha_guess = ans[1]
            beta_guess = ans[2]
            de_guess = ans[3] + self.de_ref
            da_guess = ans[4]
            dr_guess = ans[5]
            condition[0] = np.abs(self.alpha_o - alpha_guess)
            condition[1] = np.abs(self.beta_o - beta_guess)
            condition[2] = np.abs(self.de_o - de_guess)
            condition[3] = np.abs(self.da_o - da_guess)
            condition[4] = np.abs(self.dr_o - dr_guess)
            self.alpha_o = alpha_guess
            self.beta_o = beta_guess
            self.de_o = de_guess
            self.da_o = da_guess
            self.dr_o = dr_guess
        self.tau_o = ans[0]
        trim_settings = [self.tau_o, np.rad2deg(self.alpha_o),
                         np.rad2deg(self.beta_o), np.rad2deg(self.de_o),
                         np.rad2deg(self.da_o), np.rad2deg(self.dr_o),
                         np.rad2deg(self.elevation)]
        print("Trim settings")
        print("tau\t\t\talpha\t\tbeta\t\t\tde\t\tda")
        print("dr\t\t\ttheta")
        print(" ".join('%0.16g' % item for item in trim_settings))
        self.climb_to_elevation(self.alpha_o, self.beta_o)
        St = np.sin(self.elevation)
        Ct = np.cos(self.elevation)
        Omega = (Sp*Ct*g/(Vo*(self.alpha_o*St + Cp*Ct)))
        self.p_o = Omega*(-St)
        self.q_o = Omega*(Sp*Ct)
        self.r_o = Omega*(Cp*Ct)
        self.controls[0] = self.de_o
        self.controls[1] = self.da_o
        self.controls[2] = self.dr_o
        self.controls[3] = self.tau_o


    def climb_to_elevation(self, alpha, beta):
        SG = np.sin(self.climb)
        SP = np.sin(self.bank)
        CP = np.cos(self.bank)
        SA = np.sin(alpha)
        CA = np.cos(alpha)
        SB = np.sin(beta)
        CB = np.cos(beta)
        A = (SA*CB)/(np.sqrt(1. - (SA**2)*(SB**2)))
        B = (SB*CA)/(np.sqrt(1. - (SA**2)*(SB**2)))
        C = (CA*CB)/(np.sqrt(1. - (SA**2)*(SB**2)))
        const_outer = (B*SP + A*CP)
        const_sqrt = np.sqrt(C**2 + (B*SP + A*CP)**2 - SG**2)
        const_denom = (C**2 + (B*SP + A*CP)**2)
        theta_try_1 = np.arcsin((C*SG + const_outer*const_sqrt)/const_denom)
        theta_try_2 = np.arcsin((C*SG - const_outer*const_sqrt)/const_denom)
        condition = np.isclose((C*np.sin(theta_try_1) -
                                (B*SP + A*CP)*np.cos(theta_try_1)),
                               SG, atol=1e-13)
        if condition is True:
            self.elevation = theta_try_1
        else:
            self.elevation = theta_try_2

    def geographic_coords(self, xyz):
        delx = (xyz[0] - self.state_vars[6])
        dely = (xyz[1] - self.state_vars[7])
        delz = (xyz[2] - self.state_vars[8])
        lat1 = self.latitude
        long1 = self.longitude
        H1 = self.state_vars[8]
        CL = np.cos(lat1)
        SL = np.sin(lat1)
        d = np.sqrt(delx**2 + dely**2)
        RE = 20888146.325  # feet
        if d < 1e-14:
            self.del_heading = 0
        else:
            theta = d/(RE + H1 - (delz/2))
            CT = np.cos(theta)
            ST = np.sin(theta)
            psi_g1 = np.arctan2(dely, delx)
            CP = np.cos(psi_g1)
            SP = np.sin(psi_g1)
            x_hat = CL*CT - SL*ST*CP
            y_hat = ST*SP
            z_hat = SL*CT + CL*ST*CP
            x_hat_p = -CL*ST - SL*CT*CP
            y_hat_p = CT*SP
            z_hat_p = -SL*ST + CL*CT*CP
            r_hat = np.sqrt(x_hat**2 + y_hat**2)
            lat2 = np.arctan2(z_hat, r_hat)
            long2 = self.longitude + np.arctan2(y_hat, x_hat)
            C = x_hat**2*z_hat_p
            S = (x_hat*y_hat_p -
                 y_hat*x_hat_p)*(np.cos(lat2)**2)*(np.cos(long2 - long1))
            self.del_heading = np.arctan2(S, C) - psi_g1
            self.latitude = lat2
            self.longitude = long2
            self.heading += self.del_heading
        flat_transform = np.array([-self.state_vars[12], -self.state_vars[11],
                                   self.state_vars[10], self.state_vars[9]])
        self.geo_quat = (np.cos(self.del_heading/2)*self.state_vars[-4:] +
                         np.sin(self.del_heading/2)*flat_transform)
