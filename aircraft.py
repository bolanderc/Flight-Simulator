#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  7 16:10:15 2018

@author: christian
"""
import numpy as np
import quat


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


        """ For the arrow problem only. (11.14 in Phillips)"""
#        self.l_r = 2.0  # Reference length
#        self.rho = 2.0e-3  # Air density
#        self.k0 = 0.0061  # Force/Moment coefficients
#        self.k1 = 1.40
#        self.k2 = 0.0059
#        self.k3 = 0.0040
#        self.k4 = 0.0080
#        self.k5 = 0.00475
#        self.C_lo = 0.00495

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
        CL = (self.CL_ref + self.CL_a*alpha + self.CL_q*qbar + self.CL_de*de)
        CS = (self.CY_b*beta + self.CY_p*pbar + self.CY_r*rbar +
              self.CY_da*da + self.CY_dr*dr)
        CD = (self.C_D0 + self.C_D1*CL + self.C_D2*(CL**2) +
              self.C_D3*(CS**2) +
              self.CD_q*qbar + self.CD_de*de)
        C_l = (self.Cll_b*beta + self.Cll_p*pbar +
               (self.Cll_r/self.CL_ref)*CL*rbar + self.Cll_da*da +
               self.Cll_dr*dr)
        Cm = (self.Cm_ref +
              (self.Cm_a/self.CL_a)*(CL*u/V - self.CL_ref + CD*w/V) +
              self.Cm_q*qbar + self.Cm_de*de)
        Cn = ((self.Cn_b/self.CY_b)*(CS*u/V - CD*v/V) +
              (self.Cn_p/self.CL_ref)*CL*pbar + self.Cn_r*rbar +
              self.Cn_da*da + self.Cn_dr*dr)
        self.F_M[0] = (norm_const*(CL*np.sin(alpha) - CS*np.sin(beta) -
                       CD*u/V))  # X aerodynamic force
        self.F_M[1] = (norm_const*(CS*np.cos(beta) -
                       CD*v/V))  # Y aerodynamic force
        self.F_M[2] = (norm_const*(-CL*np.cos(alpha) -
                       CD*w/V))   # Z aerodynamic force
        self.F_M[3] = (norm_const*self.b_w*C_l)  # Aerodynamic rolling moment
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