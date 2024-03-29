from math import *
import numpy as np
from scipy.io import loadmat
from numpy import polyval
from pandas import DataFrame, concat
from scipy.interpolate import interp1d
from Skywalker_update import USkywalker2015_Aerodynamic


clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

skywalker = USkywalker2015_Aerodynamic()


class UAVstate0:
    Tet0 = 0    
    Tan0 = 1.5
    AoA0 = Tan0 - Tet0
    Va0 = 40/3.6
    H0 =  50
    L0 = 0
    de0 = 0
# П
class SAU_in:
    # Текущий угол наклона траектории
    Tet = 0
    # Текущий угол тангажа
    Tan = 0
    # Текущая угловая скорость, связанная СК, град/сек
    Wz1 = 0
    # Текущая высота
    H = 0
    # Текущая вертикальная скорость
    Vy = 0
    # 
    Vw = 0
class SAU_out:
    P = 0.1  # Положение дросселя двигателя в диапазоне от 0 до 1
    de = float(0)   # Угол отклонения руля высоты
# Опишем для продольного движения
class UAV:

    

    def reset(self):
        # Угол атаки
        self.fAoA = 0
        self.fAoADt= 0

        # Аэродинамика
        self.fXaForce = 0 
        self.fX1Force = 0
        self.fYaForce = 0
        self.fY1Force = 0
        self.fZaForce = 0
        self.fZ1Froce = 0

        
        self.fMx1Moment = 0
        self.fMy1Moment = 0
        self.fMz1Moment = 0

        self.fTime = 0
        self.fVa = 0
        self.fVaDt = 0
        self.g = 9.88

        self.fTan = 0
        self.fPsi = 0
        self.fGam = 0

        self.fTanDt = 0
        self.fPsiDt = 0
        self.fGamDt = 0

        self.fWxa = 0
        self.fWya = 0
        self.fWza = 0

        self.fWz1 = 0

        self.fTet = 0
        self.fPhi = 0
        self.fGms = 0
        
        self.fTetDt = 0
        self.fPhiDt = 0
        self.fGmsDt = 0

        self.fX = 0
        self.fY = 0
        self.fZ = 0

    #state
    #0 - Начальное значение угла наклона траектории
        

    def __init__(self,fStep,endtime,state0 = UAVstate0()):
        # Файл АДХ

        self.fEndTime = endtime
        self.fStep = fStep
        self.reset()
        # Масса
        self.m = 2.50
        # Моменты инерции 
        self.Izz = skywalker.Izz
        self.lw = 1.85 # Размах крыла(м)
        self.ba = 0.25 # САХ(м)
        self.bc = 0.25 # Корневая хорда крыла(м)
        self.Sw = self.lw * self.ba # Площадь крыла(м2)
        # Характерные площади
        self.fSx = self.Sw
        # self.fBx = 0.3
        self.fLx = self.ba

        

        # ВМГ

        self.drossel = np.linspace(0, 1, 10, endpoint=True) # Положение дросселя
        self.P_one = np.linspace(12, 30, 10, endpoint=True) # Ньютон

        # Инициализация начального состояния
      
        self.fTet = state0.Tet0
        self.fTan = state0.Tan0
        self.fAoA = state0.AoA0
        self.fVa = state0.Va0
        self.fY = state0.H0
        self.fX = state0.L0
        self.delta_lift = state0.de0

        # Матрицы перехода
        self.speed_to_earth = self.att_from_angles(self.fTet,self.fGms,self.fPhi)
        self.attach_to_earth = self.att_from_angles(self.fTan,self.fGam,self.fPsi)
        self.speed_to_attach = self.stc_from_angles(self.fAoA,0)

        # Ускорения 
        self.dV1 = np.matmul(self.attach_to_earth.transpose(),np.array([self.fVaDt,0,0]))
        # Скорости
        Va_vec = np.array([[self.fVa],[0],[0]])
        # Вектор скорости, земная СК, м/с
        self.Vg = np.matmul(self.speed_to_earth,Va_vec)
        # Истинная скорость, м/с
        self._Vg = np.linalg.norm(self.Vg)
        # Путевая скорость, м/с
        self._Vp = np.linalg.norm(np.array([[self.Vg[0]],[self.Vg[2]]]))
        # Вектор скорости, связанная СК, м/с
        self.V1 = np.matmul(self.attach_to_earth.transpose(),self.Vg)

        self.u = SAU_out()
        # Инициализация записи
        columns_names=list(["Time","H","L","Vgx","Vgy","Va","Theta","Tang","AoA","Vpr_kmh"])
        columns_names_SAU=list(["P","delta_elerons"])
        columns_names_debug=list(["Wz1","Mza","P1","Mzp"])

        columns_names_data = columns_names + columns_names_SAU + columns_names_debug

        data = [self.fTime,self.fY,self.fX,float(self.Vg[0]), float(self.Vg[1]),self.fVa,self.fTet,self.fTan,self.fAoA,self._Vp*3.6]
        data_sau = [self.u.P,self.u.de]
        
        data_debug = [0,0,0,0]
        data_all = data + data_sau + data_debug

        self.db = DataFrame([data_all],columns = columns_names_data)
        # self.data = np.array([UAV_frame_prodol()])

    def fCxa(self, AoA):
        return skywalker.calculate_cx(AoA)

    def fCya(self, AoA):
        return skywalker.calculate_cy(AoA)

      # Расчет ВМГ
    def calc_VMG(self,dr):
        dr = np.clip(dr,0,1)
        return np.interp(dr,self.drossel,self.P_one)
    # Расчет плотности
    def compute_dynamic_pressure(self, H, V):
        rho = 1.225*(1 - 2.2569e-05*H)**(4.2586)
        q = 0.5*rho*V**2
        return rho,q
    def calculate_mz(self,AoA,cya,V, AoA_dot, wz1, lift):
        return skywalker.calculate_moments( AoA,cya,V, AoA_dot, wz1, lift)   
    
    
  


    def att_from_angles(self,pitch,roll,yaw):

        sin_tet = sin(radians(pitch))
        cos_tet = cos(radians(pitch))
        sin_gamma = sin(radians(roll))
        cos_gamma = cos(radians(roll))
        sin_psi = sin(radians(yaw))
        cos_psi = cos(radians(yaw))


        C11 = cos_psi * cos_tet
        C12 = sin_tet
        C13 = -sin_psi *cos_tet

        C21 = sin_psi * sin_gamma - cos_psi * sin_tet * cos_gamma
        C22 = cos_tet *cos_gamma
        C23 = sin_gamma *cos_psi + sin_psi * sin_tet *cos_gamma

        C31 = sin_psi*cos_gamma + cos_psi*sin_tet*sin_gamma
        C32 = -cos_tet *sin_gamma
        C33 = cos_psi * cos_gamma - sin_psi * sin_tet * sin_gamma

        out = np.array([[C11,C21,C31],[C12,C22,C32],[C13,C23,C33]])

        return out
    
    def stc_from_angles(self,AoA_deg,AoS_deg):
        SinA = sin(radians(AoA_deg))
        SinS = sin(radians(AoS_deg))
        CosA = cos(radians(AoA_deg))
        CosS = cos(radians(AoS_deg))

        C11 = CosA * CosS
        C12 = SinA
        C13 = -CosA*SinS

        C21 = -SinA*CosS
        C22 = CosA
        C23 = SinA*SinS

        C31 = SinS
        C32 = 0
        C33 = CosS
        out = np.array([[C11,C12,C13],[C21,C22,C23],[C31,C32,C33]])
        return out
    
    # Расчет производных углов Эйлера методом Эйлера
    
    def calc_derivative_Euler(self):
       
        Tet_rad = radians(self.fTet)
        Gms_rad = radians(self.fGms)
        self.fTetDt = degrees((self.fWya * sin(Gms_rad)+self.fWza * cos(Gms_rad)))
        self.fPsiDt = degrees((1/cos(Tet_rad)) * (self.fWya*cos(Gms_rad)-self.fWza*sin(Gms_rad)))
        self.fGmsDt = degrees((self.fWxa - (self.fWya*cos(Gms_rad)-self.fWza*sin(Gms_rad)))*tan(Tet_rad))
    def write_DataFrame(self):
        columns_names=list(["Time","H","L","Vgx","Vgy","Va","Theta","Tang","AoA","Vpr_kmh"])
        columns_names_SAU=list(["P", "delta_elerons"])
        columns_names_debug=list(["Wz1","Mza","P1"])
        columns_names_data = columns_names + columns_names_SAU +columns_names_debug

        data = [self.fTime,self.fY,self.fX,float(self.Vg[0]), float(self.Vg[1]),self.fVa,self.fTet,self.fTan,self.fAoA,self._Vp*3.6]
        data_sau = [self.u.P, self.u.de]
        data_debug = [self.fWz1,self.Mza,self.P1]
        data_all = data + data_sau + data_debug

        frame = DataFrame([data_all],columns = columns_names_data)
        self.db = concat([self.db, frame], ignore_index=True)
    # Расчет правых частей дифф. уравнений
    def step(self, u = SAU_out()):

        # Вектор управления извне
        self.u = u
        # Тяга от дросселя, Н
        P1 = self.calc_VMG(u.P)

        # На запись
        self.P1 = P1

        # Элероны, град.
        self.delta_elerons = u.de

        # Тяга в связанной СК

        self.Px1 = P1

        # Расчет плотности на текущей выосте и скорости
        # Расчет напора набегающего потока
        self.rho,self.fQ = self.compute_dynamic_pressure(self.fY,self.fVa)
       
        # Угол атаки, град.
        self.fAoA =  self.fTan - self.fTet# degrees(atan2(-self.V1[1],self.V1[0]))
        # Вес,Н
        self.fG = self.m*self.g
        # Матрицы перехода
        self.speed_to_earth = self.att_from_angles(self.fTet,self.fGms,self.fPhi)
        self.attach_to_earth = self.att_from_angles(self.fTan,self.fGam,self.fPsi)
        self.speed_to_attach = self.stc_from_angles(self.fAoA,0)
        # Вектор сил тяжести
        self.Gxyz = np.matmul(self.speed_to_earth.transpose(),np.array([0,self.fG,0]))
        # Силы в связанной СК
        # ДУ
        self.Fxyz_a_prop = np.matmul(self.speed_to_attach.transpose(),np.array([self.Px1,0, 0]))
        # Силы в скоростной СК
        # Аэродинамика, силы
        # Лобовое сопротивление
        self.fXaForce = -self.fCxa(self.fAoA) * self.fQ * self.fSx
        # Подъемная сила
        self.fYaForce = self.fCya(self.fAoA) * self.fQ * self.fSx
        # Ускорение в земной СК
        self.fVaDt = float((self.Fxyz_a_prop[0] + self.fXaForce - self.Gxyz[0])/self.m)

        if (np.nonzero(self.fVa)):
            self.fWza = float((self.Fxyz_a_prop[1] + self.fYaForce - self.Gxyz[1])/(self.m * self.fVa))
            self.fWya = 0 # (-self.fZaForce + self.Gxyz[2])/(self.m * self.fVa)
        else:
            self.fWza = float(0)
            self.fWya = float(0)    
        self.fWxa = float(0)#((self.fWx1*CosA - self.fWy1 * SinA) + self.fWza * SinS)/CosS




        # Скорости
        Va_vec = np.array([[self.fVa],[0],[0]])
        # Вектор скорости, земная СК, м/с
        self.Vg = np.matmul(self.speed_to_earth,Va_vec)
        # Истинная скорость, м/с
        self._Vg = np.linalg.norm(self.Vg)
        # Путевая скорость, м/с
        self._Vp = np.linalg.norm(np.array([[self.Vg[0]],[self.Vg[2]]]))
        # Вектор скорости, связанная СК, м/с
        self.V1 = np.matmul(self.attach_to_earth.transpose(),self.Vg)
        # Расчет перегрузок
        n_dot = np.array([float(self.fVaDt/self.g),float((self.fVa/self.g)*self.fWza),float(-(self.fVa/self.g)*self.fWya)])
        n_dot_e = np.matmul(self.speed_to_earth.transpose(),(np.array([0,1,0])))
        # Вектор перегрузок, скоростная СК
        self.na = n_dot+n_dot_e
        # Вектор перегрузок, связанная СК
        self.n1 = np.matmul(self.speed_to_attach,self.na)
        
        # Моменты, от аэродинамики
        self.cx = self.fCxa(self.fAoA)
        self.cy = self.fCya(self.fAoA)
        Mza = self.calculate_mz(self.fAoA,0,1,0,0,0) * self.fQ * self.fLx * self.fSx
        self.Mza = Mza


        # Так как только продольный канал, моменты Mx1 и My1 не считаем
        self.fMx1Moment = 0
        self.fMy1Moment = 0
        self.fMz1Moment = Mza


        # Моменты, связанная СК

        self.fWx1Dt = 0#(self.fMx1Moment + (self.fIyy - self.fIzz)*self.fWy1*self.fWz1)  / self.fIxx
        self.fWy1Dt = 0#(self.fMy1Moment + (self.fIzz - self.fIxx)*self.fWx1*self.fWz1)  / self.fIyy
        self.fWz1Dt = self.fMz1Moment/self.Izz #(self.fMz1Moment - (self.fIyy - self.fIxx)*self.fWx1*self.fWy1)  / self.fIzz
		
        # Производные углов Эйлера, тк у нас только продольное то это просто wz
        
        self.fTetDt = self.fWza
        self.fWz1 = self.fTanDt = self.fTanDt + self.fWz1Dt*self.fStep
        
        self.fTet = radians(self.fTet) + self.fTetDt*self.fStep
        self.fTan = radians(self.fTan) + self.fTanDt*self.fStep

        self.fTet = degrees(self.fTet)
        self.fTan = degrees(self.fTan)

        self.fVa = self.fVa + self.fVaDt*self.fStep

        self.fX = self.fX + self.Vg[0] * self.fStep
        self.fY = self.fY + self.Vg[1] * self.fStep

        # Расчет производной угла атаки, рад.
        Vx1 = self.V1[0]
        Vy1 = self.V1[1]

        dVx1 = self.dV1[0]
        dVy1 = self.dV1[1]

        self.fAoADt = -(Vx1 * dVx1 - Vy1 * dVy1) / (Vx1 * Vx1 + Vy1 * Vy1)

        self.fTime = self.fTime + self.fStep



        # Записываем данные
        self.write_DataFrame()

    def get_SAU_data(self):
        ans = SAU_in()
        ans.Tet = self.fTet
        ans.H = self.fY
        ans.Wz1 = degrees(self.fTanDt)
        ans.Tan = self.fTan
        ans.Vw = self._Vg
        ans.Vy = self.Vg[1]
        return ans
