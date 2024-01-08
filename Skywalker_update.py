import numpy as np
from scipy.optimize import minimize
import math


class USkywalker2015_Aerodynamic():
      def __init__(self):
        # Constants
        self.g = 9.81  # Acceleration due to gravity (m/s^2)
        self.m = 2.50  # Mass of the aircraft (kg)
        self.mu = 1.785e-5  # Dynamic viscosity of air (kg/m*s)
        self.rho = 1.225  # Density of air (kg/m^3)
        self.pi = 3.14

        # Wing
        self.lw = 1.85  # Wing span (m)
        self.ba = 0.25  # Wing aspect ratio
        self.bc = 0.25  # Wing chord ratio
        self.Sw = self.lw * self.ba  # Wing area (m^2)

        # Other variables
        self.lambda0 = self.lw * self.lw / self.Sw
        self.khi = 1.2
        self.psi_V = 0
        self.eta = self.bc / self.ba
        self.zst_ = (self.eta + 2) / (self.eta + 1) / 3
        self.ksi2 = (self.eta + 3) / (self.eta + 1) / 6
        self.ki = 0.9 * (0.5 + 0.033 * self.lambda0)

        # Fuselage
        self.lf = 0.60  # Fuselage length (m)
        self.hf = 0.25  # Fuselage height (m)
        self.wf = 0.125  # Fuselage width (m)
        self.Sf = 0.8 * self.wf * self.hf  # Fuselage area (m^2)
        self.Swf = self.wf * self.ba  # Fuselage wetted area (m^2)
        self.kint = 0.95
        self.lambdaf = self.lf / math.sqrt(4 * self.Sf / self.pi)
        self.xtf = 0.38
        self.xf_ = self.xtf / self.lf
        self.cxf = 0.11
        self.czbf = -0.28 * 0.8 * self.lf * self.hf / self.Sw / 57.3

        # Engine
        self.num_eng = 1
        self.D = 12.5 * 2.54 / 100
        self.F = self.pi * self.D * self.D / 4
        self.phi = 5
        self.xv = -0.25
        self.yp = 0.05

        # Landing gear
        self.lgo = 0.535
        self.Lgo = 0.85
        self.bgo = 0.195
        self.khigo = 10
        self.Sgo = self.lgo * (self.bgo - 0.25 * self.lgo * math.tan(self.khigo / 180 * self.pi))
        self.Sfgo = 0
        self.lv = 0.45
        self.bv = 0.06
        self.Sv = self.bv * self.lv
        self.nv = math.sqrt(self.Sv / self.Sgo)
        self.k = 0.93 - 0.25 * self.Sfgo / self.Sgo
        self.k_sigma = 0.75 * self.D / self.lgo

        # Vertical tail
        self.lvo = 0.175
        self.Lvo = 0.85
        self.bvo = 0.2
        self.khivo = 30
        self.Svo = 0.034
        self.yvo = 0.1
        self.bn = 0.075
        self.Sn = self.bn * self.lvo
        self.khin = 5
        self.nn = math.sqrt(self.Sn / self.Svo) * math.cos(self.khin / 57.3)

        # Horizontal tail
        self.be = 0.06
        self.Le = 1.15
        self.le = 0.35
        self.Sobe = 2 * self.le * self.ba
        self.khie = 0
        self.n_e = self.be / self.ba * math.cos(self.khie / 57.3)
        self.ke = 0.6 + 0.066 * (self.eta - 1)

        # Предположительно
        # self.Q = self.compute_dynamic_pressure(H, V)
        self.P = 850
        self.k = 1
        # self.B = self.P / self.Q / self.F;
        self.B = 1
        self.kefvo = self.k * (1 + self.B)


        # Wing parameters
        self.xf = 0.25 * self.ba
        self.xt = 0.25 * self.ba

        # Wing coefficients
        self.cx0_wing = 0.012896
        self.cx1_wing = 0.00098335
        self.cx2_wing = 0.00037203
        self.cy0_wing = 0.26531
        self.cy1_wing = 0.079116
        self.cz0_wing = -1.2273e-06
        self.cz1_wing = -0.00019605
        self.mx0_wing = -5.8182e-06
        self.mx1_wing = -0.00051789
        self.my0_wing = -2.7273e-07
        self.my1_wing = -4.3636e-05
        self.mz0_wing = -0.088957
        self.mz1_wing = -0.0016479
        self.mz2_wing = -8.3762e-05

        # Tail coefficients
        self.cx0_tail = 0.0024427
        self.cx1_tail = -3.4293e-05
        self.cx2_tail = 6.7229e-05
        self.cy0_tail = -0.011916
        self.cy1_tail = 0.0076309
        self.cz0_tail = -4.4545e-06
        self.cz1_tail = -0.0024551
        self.mx0_tail = 6.5909e-06
        self.mx1_tail = 4.2364e-06
        self.my0_tail = -1.1136e-05
        self.my1_tail = -0.0010835
        self.mz0_tail = 0.039042
        self.mz1_tail = -0.025858
        self.mz2_tail = -1.5002e-05

        # Moments of inertia
        self.Ixx = 0.16893
        self.Iyy = 0.19021
        self.Izz = 0.34630
        self.Ixy = -0.01131

        self.mzwz = 0
        self.mza = 0
        self.mzdv = 0
        self.mzadot = 0

        self.mxwx = 0
        self.mxwy = 0
        self.mxb = 0
        self.mxde = 0
        self.mzf = 0
        self.mzgd = 0
        self.mzp = 0
        self.mzpp = 0
        self.mzp = 0

        self.mywx = 0
        self.mywy = 0
        self.myb = 0
        self.myde = 0
        self.mydn = 0
        

      def compute_dynamic_pressure(aerodynamic_obj, H, V):
        rho = 1.225*(1 - 2.2569e-05*H)**4.2586
        q = 0.5*rho*V**2
        return q

      def calculate_cx(self,AoA):
        cx0_wing_calc = self.cx0_wing * (1 - self.kint * self.Swf / self.Sw)
        cx1_wing_calc = self.cx1_wing * (1 - self.kint * self.Swf / self.Sw)
        cx2_wing_calc = self.cx2_wing * (1 - self.kint * self.Swf / self.Sw)

        cx0_tail_calc = self.cx0_tail * (1 - self.kint * self.Sfgo / self.Sgo) + 0.00175
        cx1_tail_calc = self.cx1_tail * (1 - self.kint * self.Sfgo / self.Sgo)
        cx2_tail_calc = self.cx2_tail * (1 - self.kint * self.Sfgo / self.Sgo)

        cx0 = 1.02 * (cx0_wing_calc + cx0_tail_calc + self.cxf)
        cx1 = 1.02 * (cx1_wing_calc + cx1_tail_calc)
        cx2 = 1.02 * (cx2_wing_calc + cx2_tail_calc)

        AoA_rad = math.radians(AoA)
        return cx0 + cx1 * AoA + cx2 * AoA * AoA

      def calculate_cy(self,AoA):
        cy0 = self.cy0_wing + self.cy0_tail
        cya = self.cy1_wing + self.cy1_tail
        AoA_rad = math.radians(AoA)
        return 1 * (cy0 + 2 * cya * AoA_rad)

      def calculate_cz(self, AoS):
        czb = self.cz1_wing + self.cz1_tail + self.czbf
        return czb * AoS

      def calculate_moments(self, AoA, Cx, Cy, wz1, lift):
        V = 10
        mz0_bgo = self.mz0_wing + self.mzf + self.mzgd + self.mzp + self.mzpp
        mz1_bgo = self.mz1_wing
        mz2_bgo = self.mz2_wing

        mxb_tail = self.kefvo * self.mx1_tail
        myb_tail = self.kefvo * self.my1_tail

        kefgo = self.k * (1 + self.k_sigma * self.B)
        mz0_go = kefgo * self.mz0_tail
        mz1_go = kefgo * self.mz1_tail
        mz2_go = kefgo * self.mz2_tail

        cy0 = self.cy0_wing + self.cy0_tail
        cya = self.cy1_wing + self.cy1_tail

        cy1 = Cx * math.sin(AoA / 57.3) + Cy * math.cos(AoA / 57.3);
        cx1 = Cx * math.cos(AoA / 57.3) - Cy * math.sin(AoA / 57.3);
        mz0 = mz0_bgo + mz0_go
        mza = (mz1_bgo + mz1_go) + 2 * (mz2_bgo + mz2_go) * AoA
        mzago = (2 * self.mz2_tail * AoA + self.mz1_tail)
        mzdv = kefgo * mzago * self.nv
        mz_go_wz_ = math.sqrt(kefgo) * mzago * self.Lgo / self.ba
        mz_gof_wz_ = 1.2 * mz_go_wz_
        mz_wing_wz_ = -0.25 * cya * 57.3 * pow((1 - 2 * self.xt / self.ba), 2) - (2 * math.pi - cya * 57.3) / 16
        mz_wing_wz_ = mz_wing_wz_ / 57.3
        mzwz_ = mz_gof_wz_ + mz_wing_wz_
        mzwz = mzwz_ * self.ba / V
        D_func = 0.1206 + 0.1814 / self.eta + 0.0980 / self.eta * self.eta;
        mzadot_ = math.sqrt(kefgo) * mzago * self.Lgo / self.ba * D_func * cya;
        mzadot = mzadot_ * self.ba / V
        mz = mz0 * 0 + mzwz * wz1 + mza * AoA + lift * mzdv + mzadot * 0 * 57.3

        
        return mz
      def simulate(self, Theta, Psi, gammaa):
        dXg = Vk * np.cos(Theta) * np.cos(Psi)
        dYg = Vk * np.sin(Theta)
        dZg = -Vk * np.cos(Theta) * np.sin(Psi)

        dVk = Rx / self.m - self.g * np.sin(Theta)
        dTheta = (Ry * np.cos(gammaa) - Rz * np.sin(gammaa) - m * g * np.cos(Theta)) / m / Vk
        dPsi = -(Ry * np.sin(gammaa) + Rz * np.cos(gammaa)) / m / Vk / np.cos(Theta)

        dalpha = wz + np.tan(beta) * (wy * np.sin(alpha) - wx * np.cos(alpha)) - (
                    Ry - m * g * np.cos(Theta) * np.cos(gammaa)) / m / Vk / np.cos(beta)
        dbeta = wx * np.sin(alpha) + wy * np.cos(alpha) + (
                    Rz + m * g * np.cos(Theta) * np.sin(gammaa)) / m / Vk

        dpsi = (wy * np.cos(gamma) - wz * np.sin(gamma)) / np.cos(theta)
        dtheta = wy * np.sin(gamma) + wz * np.cos(gamma)
        dgamma = (wz * np.sin(gamma) - wy * np.cos(gamma)) * np.tan(theta) + wx

        dwx = (Iyy * MRx + Ixy * MRy + Ixy * (Izz - Ixx - Iyy) * wx * wy + (
                    Iyy * (Iyy - Izz) + Ixy ** 2) * wy * wz) / (Ixx * Iyy - Ixy ** 2)
        dwy = (Ixx * MRy + Ixy * MRx + Ixy * (Ixx + Iyy - Izz) * wy * wz + (
                    Ixx * (Izz - Ixx) - Ixy ** 2) * wx * wy) / (Ixx * Iyy - Ixy ** 2)
        dwz = (MRz + (Ixx - Iyy) * wx * wy + Ixy * (wx ** 2 - wy ** 2)) / Izz

        # Return derivatives
        return [dXg, dYg, dZg, dVk, dTheta, dPsi, dalpha, dbeta, dpsi, dtheta, dgamma, dwx, dwy, dwz, d_tang]
      def get_parameters(self):
        parameters = [
            self.lambda0, self.khi, self.psi_V, self.eta, self.zst_,
            self.ksi2, self.ki, self.lf, self.hf, self.wf, self.lgo,
            self.Lgo, self.bgo, self.khigo, self.lv, self.bv, self.lvo,
            self.Lvo, self.bvo, self.khivo, self.Svo, self.yvo, self.bn,
            self.be, self.Le, self.le, self.Sw, self.cx0_wing, self.cx1_wing,
            self.cx2_wing, self.cy0_wing, self.cy1_wing, self.cz0_wing,
            self.cz1_wing, self.mx0_wing, self.mx1_wing, self.my0_wing,
            self.my1_wing, self.mz0_wing, self.mz1_wing, self.mz2_wing,
            self.cx0_tail, self.cx1_tail, self.cx2_tail, self.cy0_tail,
            self.cy1_tail, self.cz0_tail, self.cz1_tail, self.mx0_tail,
            self.mx1_tail, self.my0_tail, self.my1_tail, self.mz0_tail,
            self.mz1_tail, self.mz2_tail, self.Ixx, self.Iyy, self.Izz,
            self.Ixy, self.mzwz, self.mza, self.mzdv, self.mzadot,
            self.mxwx, self.mxwy, self.mxb, self.mxde, self.mzf, self.mzgd,
            self.mzp, self.mzpp, self.mzp, self.mywx, self.mywy, self.myb,
            self.myde, self.mydn
        ]
        return np.array(parameters)

        # def set_parameters(self, parameters):
        # (
        #     self.lambda0, self.khi, self.psi_V, self.eta, self.zst_,
        #     self.ksi2, self.ki, self.lf, self.hf, self.wf, self.lgo,
        #     self.Lgo, self.bgo, self.khigo, self.lv, self.bv, self.lvo,
        #     self.Lvo, self.bvo, self.khivo, self.Svo, self.yvo, self.bn,
        #     self.be, self.Le, self.le, self.Sw, self.cx0_wing, self.cx1_wing,
        #     self.cx2_wing, self.cy0_wing, self.cy1_wing, self.cz0_wing,
        #     self.cz1_wing, self.mx0_wing, self.mx1_wing, self.my0_wing,
        #     self.my1_wing, self.mz0_wing, self.mz1_wing, self.mz2_wing,
        #     self.cx0_tail, self.cx1_tail, self.cx2_tail, self.cy0_tail,
        #     self.cy1_tail, self.cz0_tail, self.cz1_tail, self.mx0_tail,
        #     self.mx1_tail, self.my0_tail, self.my1_tail, self.mz0_tail,
        #     self.mz1_tail, self.mz2_tail, self.Ixx, self.Iyy, self.Izz,
        #     self.Ixy, self.mzwz, self.mza, self.mzdv, self.mzadot,
        #     self.mxwx, self.mxwy, self.mxb, self.mxde, self.mzf, self.mzgd,
        #     self.mzp, self.mzpp, self.mzp, self.mywx, self.mywy, self.myb,
        #     self.myde, self.mydn
        # ) = parameters

      
        def objective(self, params, t_data, X_data, Y_data, Z_data, Yaw_data, Pitch_data, Roll_data, H_data, L_data, V_data,
                  lat_data, lon_data, tan_data, wx_data, wy_data, wz_data, Vx_data, Vy_data, Vz_data):
          self.set_parameters(params)
          # Моделирование движения
          X_model, Y_model, Z_model, Yaw_model, Pitch_model, Roll_model, H_model, L_model, V_model, \
          lat_model, lon_model, tan_model, wx_model, wy_model, wz_model, Vx_model, Vy_model, Vz_model = self.simulate(t_data)

          # Разница между данными и моделью
          error = np.concatenate([
              X_data - X_model,
              Y_data - Y_model,
              Z_data - Z_model,
              Yaw_data - Yaw_model,
              Pitch_data - Pitch_model,
              Roll_data - Roll_model,
              H_data - H_model,
              L_data - L_model,
              V_data - V_model,
              lat_data - lat_model,
              lon_data - lon_model,
              tan_data - tan_model,
              wx_data - wx_model,
              wy_data - wy_model,
              wz_data - wz_model,
              Vx_data - Vx_model,
              Vy_data - Vy_model,
              Vz_data - Vz_model
          ])

          return np.sum(error ** 2)

        def optimize(self, t_data, X_data, Y_data, Z_data, Yaw_data, Pitch_data, Roll_data, H_data, L_data, V_data,
                 lat_data, lon_data, tan_data, wx_data, wy_data, wz_data, Vx_data, Vy_data, Vz_data):
          # Начальные значения параметров
          initial_params = self.get_parameters()

          # Оптимизация
          result = minimize(self.objective, initial_params,
                            args=(t_data, X_data, Y_data, Z_data, Yaw_data, Pitch_data, Roll_data, H_data, L_data, V_data,
                                  lat_data, lon_data, tan_data, wx_data, wy_data, wz_data, Vx_data, Vy_data, Vz_data),
                            method='L-BFGS-B')

          # Получаем оптимальные параметры
          optimized_params = result.x
          self.set_parameters(optimized_params)
          return optimized_params

#  данные телеметрии
t_data = [0.540681, 1.01617, 1.524803, 2.025008, 2.528205, 3.033129, 3.529344, 4.024273, 4.528166, 5.03059, 5.520191, 6.021328, 6.521206, 7.024735, 7.522662, 8.016132, 8.523443, 9.021043, 9.519763, 10.020839, 10.523409, 11.019663, 11.516958, 12.019306, 12.524587, 13.019237, 13.522451, 14.017725, 14.520527, 15.021486, 15.52177, 16.021942, 16.523697, 17.031635, 17.526648, 18.015903, 18.532206, 19.01767, 19.520054, 20.028341, 20.519499, 21.017576, 21.524736, 22.02195, 22.532293, 23.020821, 23.519682, 24.017954, 24.52302, 25.032635, 25.518934, 26.026217, 26.522844]
X_data = [106.062065, 115.914452, 126.499092, 136.497269, 146.122406, 155.374771, 164.064667, 172.335693, 180.369873, 188.025131, 195.181915, 202.236633, 209.047791, 215.736404, 222.230957, 228.591324, 235.085144, 241.43515, 247.798035, 254.201614, 260.642792, 267.024048, 273.439667, 279.940399, 286.496399, 292.928467, 299.483093, 305.942566, 312.510742, 319.140564, 325.974243, 333.103394, 340.595581, 348.539825, 356.620392, 364.914337, 373.961273, 382.703613, 391.950256, 401.475525, 410.814209, 420.397675, 430.25705, 440.009155, 450.096893, 459.819611, 469.807831, 479.839447, 490.059753, 500.420837, 510.350952, 520.751282, 530.972046]
Y_data = [-0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0]
Z_data = [99.937897, 99.832008, 100.827065, 102.918579, 105.186005, 107.377846, 109.450714, 111.442383, 113.393143, 115.264252, 117.022255, 118.763901, 120.449593, 122.080666, 123.624756, 125.108185, 126.59108, 128.035553, 129.481369, 130.931702, 132.395676, 133.868561, 135.355515, 136.863037, 138.375839, 139.885635, 141.314163, 142.747009, 144.169586, 145.561218, 146.676117, 147.503708, 148.056076, 148.454742, 148.723877, 148.951401, 149.150558, 149.303223, 149.421478, 149.513092, 149.582748, 149.634369, 149.670639, 149.69429, 149.707626, 149.712311, 149.709595, 149.700577, 149.686417, 149.666183, 149.637619, 149.600632, 149.558167]
Yaw_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Pitch_data = [-0.051609, 4.764829, 12.356406, 14.382731, 14.711987, 15.252884, 15.90225, 16.606873, 17.344267, 18.05895, 18.727251, 19.270838, 19.542461, 19.66621, 19.716658, 19.73646, 19.74165, 19.741758, 19.742735, 19.743896, 19.74654, 19.749853, 19.753656, 19.757435, 19.76083, 19.763868, 19.766258, 19.76815, 18.424694, 15.042863, 11.875554, 9.473124, 7.611221, 6.110348, 4.955063, 4.060377, 3.333706, 2.815438, 2.40911, 2.098576, 1.868562, 1.685274, 1.53613, 1.417442, 1.316136, 1.233785, 1.161924, 1.099032, 1.042615, 0.991661, 0.947593, 0.905628, 0.867386]
Roll_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
H_data = [99.94062, 99.829384, 100.806053, 102.877113, 105.130707, 107.315102, 109.394554, 111.392952, 113.339127, 115.203836, 116.966049, 118.698975, 120.369225, 121.994949, 123.581947, 125.086548, 126.581711, 128.036636, 129.482468, 130.930328, 132.393646, 133.865982, 135.35437, 136.862823, 138.375641, 139.885681, 141.314575, 142.747192, 144.170441, 145.562546, 146.67688, 147.503891, 148.056076, 148.454742, 148.723877, 148.951401, 149.150558, 149.303223, 149.421478, 149.513092, 149.582748, 149.634369, 149.670639, 149.69429, 149.707626, 149.712311, 149.709595, 149.700577, 149.686417, 149.666183, 149.637619, 149.600632, 149.558167]
L_data = [100.04924, 100.006668, 100.220032, 100.727722, 101.336578, 101.936058, 102.527237, 103.10878, 103.679871, 104.239868, 104.789543, 105.329163, 105.857964, 106.375938, 106.883377, 107.380966, 107.87056, 108.348488, 108.818573, 109.275513, 109.716881, 110.143661, 110.559219, 110.956429, 111.340424, 111.710724, 112.058319, 112.394264, 112.719528, 113.032471, 113.329842, 113.612793, 113.880829, 114.134109, 114.37323, 114.595657, 114.803497, 114.998985, 115.183151, 115.356255, 115.519299, 115.672287, 115.814789, 115.947807, 116.070999, 116.184357, 116.28788, 116.38147, 116.465134, 116.538879, 116.602676, 116.656532, 116.700462, 116.734474, 116.758583]
Z_data = [99.937897, 99.832008, 100.827065, 102.918579, 105.186005, 107.377846, 109.450714, 111.442383, 113.393143, 115.264252, 117.022255, 118.763901, 120.449593, 122.080666, 123.624756, 125.108185, 126.59108, 128.035553, 129.481369, 130.931702, 132.395676, 133.868561, 135.355515, 136.863037, 138.375839, 139.885635, 141.314163, 142.747009, 144.169586, 145.561218, 146.676117, 147.503708, 148.056076, 148.454742, 148.723877, 148.951401, 149.150558, 149.303223, 149.421478, 149.513092, 149.582748, 149.634369, 149.670639, 149.69429, 149.707626, 149.712311, 149.709595, 149.700577, 149.686417, 149.666183, 149.637619, 149.600632, 149.558167]
Vx_data = [-0.324844, 0.266487, 3.389016, 4.528245, 4.432708, 4.258392, 4.074125, 3.874268, 3.660905, 3.420248, 3.156991, 2.874171, 2.57079, 2.258862, 1.942367, 1.624846, 1.313907, 1.016845, 0.748154, 0.503822, 0.307548, 0.157653, 0.046107, -0.025528, -0.083473, -0.130947, -0.177201, -0.217297, -0.247768, -0.264885, -0.277358, -0.294654, -0.315645, -0.339783, -0.366919, -0.396633, -0.428578, -0.462146, -0.496605, -0.531665, -0.566678, -0.60184, -0.637146, -0.672445, -0.707414, -0.741699, -0.775003, -0.807961, -0.840209, -0.871419, -0.901289, -0.929534, -0.955898, -0.980154, -1.002125, -1.021693]
Vy_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Vz_data = [-0.040127, -0.358666, 0.218667, -0.143842, -0.390074, -0.370141, -0.370929, -0.381144, -0.405105, -0.424364, -0.442066, -0.458209, -0.475234, -0.491873, -0.507906, -0.523051, -0.538461, -0.552968, -0.568481, -0.584606, -0.602494, -0.621579, -0.641708, -0.662177, -0.681946, -0.700574, -0.718181, -0.734353, -0.749352, -0.763001, -0.775314, -0.786637, -0.797397, -0.808024, -0.819003, -0.830932, -0.844144, -0.858685, -0.874429, -0.891198, -0.90878, -0.926824, -0.944875, -0.962651, -0.979868, -0.996279, -1.01169, -1.025981, -1.039215, -1.051453, -1.062877, -1.073577, -1.08361, -1.09293, -1.101384, -1.108741, -1.114772]
wx_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
wy_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
wz_data = [4.832259, 26.018454, 6.368633, 3.374022, 2.074454, 1.501304, 1.602136, 1.203643, 1.543225, 1.387579, 1.306834, 0.754106, 0.385947, 0.150492, 0.070516, 0.038872, -0.008124, -0.005226, 0.006674, 0.001322, 0.004454, 0.009795, -0.001136, 0.01284, 0.000289, 0.00367, 0.0056, 0.002602, -6.282673, -6.673505, -5.931009, -3.964227, -3.411065, -2.637851, -2.091027, -1.697386, -1.223837, -0.960531, -0.7443, -0.551216, -0.411321, -0.323145, -0.264934, -0.211546, -0.183811, -0.158363, -0.135056, -0.119184, -0.105268, -0.096556, -0.085186, -0.07584, -0.073864]
Theta_data = [-0.911966, 0.728911, 9.392754, 13.039954, 13.304004, 13.35806, 13.478374, 13.597798, 13.693122, 13.772177, 13.825483, 13.826017, 13.645694, 13.353132, 13.069826, 12.850669, 12.708303, 12.643473, 12.640887, 12.683348, 12.753953, 12.834357, 12.915442, 12.990273, 13.055585, 13.10842, 13.150783, 13.182204, 12.748201, 10.256673, 7.346358, 5.159032, 3.714271, 2.705303, 1.997655, 1.483652, 1.080777, 0.803527, 0.593165, 0.440756, 0.334273, 0.25738, 0.200739, 0.160001, 0.129809, 0.107529, 0.090447, 0.07757, 0.067447, 0.059513, 0.053431, 0.048257, 0.044035]
Psi_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Gam_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
V_data = [20.411251, 20.949238, 20.767393, 20.070753, 19.264177, 18.434124, 17.607941, 16.80994, 16.047169, 15.365525, 14.777736, 14.257028, 13.835613, 13.516763, 13.301197, 13.167191, 13.09559, 13.0718, 13.082335, 13.115328, 13.161051, 13.209363, 13.256075, 13.298143, 13.334182, 13.362891, 13.385507, 13.401859, 13.430934, 13.620114, 14.037686, 14.62766, 15.304196, 15.995967, 16.646488, 17.236471, 17.779661, 18.210192, 18.577332, 18.882704, 19.128309, 19.339985, 19.528769, 19.688675, 19.835596, 19.960905, 20.077085, 20.183189, 20.282465, 20.376036, 20.459404, 20.541016]
lat_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
lon_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tan_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
psi_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
de_data = [-2.38, 2.221444, -2.824676, -0.229218, 0.008013, -0.960053, -2.333033, -2.793258, -3.20921, -4.096284, -4.689257, -5.449375, -6.09535, -6.673894, -7.214021, -7.50669, -7.727847, -7.860829, -7.867254, -7.837928, -7.747189, -7.656967, -7.561542, -7.475642, -7.385769, -7.320013, -7.269345, -7.230631, -6.583617, -5.324873, -4.856799, -4.524757, -4.096138, -3.455241, -2.975039, -2.445946, -2.087259, -1.769843, -1.539356, -1.372861, -1.259549, -1.158675, -1.071742, -1.008822, -0.941044, -0.894756, -0.845587, -0.80647, -0.766278, -0.730031, -0.697565, -0.665452, -0.636061]

dn_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

dv_data = [-2.38, 2.221444, -2.824676, -0.229218, 0.008013, -0.960053, -2.333033, -2.793258, -3.20921, -4.096284, -4.689257, -5.449375, -6.09535, -6.673894, -7.214021, -7.50669, -7.727847, -7.860829, -7.867254, -7.837928, -7.747189, -7.656967, -7.561542, -7.475642, -7.385769, -7.320013, -7.269345, -7.230631, -6.583617, -5.324873, -4.856799, -4.524757, -4.096138, -3.455241, -2.975039, -2.445946, -2.087259, -1.769843, -1.539356, -1.372861, -1.259549, -1.158675, -1.071742, -1.008822, -0.941044, -0.894756, -0.845587, -0.80647, -0.766278, -0.730031, -0.697565, -0.665452, -0.636061]

P_data = [11.834788, 10.636348, 9.771808, 9.075868, 8.525831, 8.124835, 7.958064, 8.156153, 8.61975, 9.272555, 9.87734, 10.601711, 11.372569, 12.063894, 12.678323, 13.164457, 13.596635, 13.92125, 14.181871, 14.378358, 14.491726, 14.553318, 14.580089, 14.580089, 14.565077, 14.541475, 14.513651, 14.485151, 14.458318, 14.404249, 14.238405, 13.891435, 13.405557, 12.717886, 12.009583, 11.211503, 10.341636, 9.553726, 8.901709, 8.375062, 8.007357, 7.742551, 7.545868, 7.402565, 7.291126, 7.207249, 7.149729, 7.107158, 7.076346, 7.055204, 7.040498, 7.029246, 7.021235]



# Загружаем данные
t_data = np.array(t_data)
X_data = np.array(X_data)
Y_data = np.array(Y_data)
Z_data = np.array(Z_data)
Yaw_data = np.array(Yaw_data)
Pitch_data = np.array(Pitch_data)
Roll_data = np.array(Roll_data)
H_data = np.array(H_data)
L_data = np.array(L_data)
V_data = np.array(V_data)
lat_data = np.array(lat_data)
lon_data = np.array(lon_data)
tan_data = np.array(tan_data)
wx_data = np.array(wx_data)
wy_data = np.array(wy_data)
wz_data = np.array(wz_data)
Vx_data = np.array(Vx_data)
Vy_data = np.array(Vy_data)
Vz_data = np.array(Vz_data)

# Создаем экземпляр модели
aerodynamic_model = USkywalker2015_Aerodynamic()

# Получаем оптимальные параметры
# optimized_params = result.x
# print("Оптимизированные параметры:")
# print(optimized_params)