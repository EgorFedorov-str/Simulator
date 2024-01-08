from UAV import SAU_in,SAU_out
from math import *
from pandas import DataFrame, concat

clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

class SAU:
    def __init__(self):
        self.u_output = SAU_out()
        self.TanU = 0
        self.TanUDt = 0
        self.delta_elerons = 0
        self.P_reg = 0
        self.Integral_H = 0
        self.dt = 0.1
        self.Time = 0
        self.writenames =list(["Time","TanU","TanU_dt","P_reg"])
        data_sau = [self.Time,self.TanU,self.TanUDt,self.P_reg]
        self.db = DataFrame([data_sau],columns = self.writenames)
    # Функция записи данных
    def writeframe(self):
        data_sau = [self.Time,self.TanU,self.TanUDt,self.P_reg]
        frame = DataFrame([data_sau],columns = self.writenames)
        self.db = concat([self.db, frame], ignore_index=True)
    # Функция вычисления текущего положения дросселя в зависимости от заданной приборной скорости
    def calc_drossel(self,V_zad):
        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        k_P = 0.5 # Пропорциональный коэффициент
        self.P_reg = 0.1 + k_P*(-self.Vw + V_zad)
        # ограничиваем
        self.drossel = clamp(self.P_reg,0.1,1)
    def calc_delta_elerons(self):
        # Пропорциональное звено
        k_P = 1.5
        # Интегральное звено
        k_i = 0.05
        # Дифференциальное звено
        k_d = 0.5

        # Заданный угол тангажа в зависимости от высоты, с ограничением
        self.TanU = clamp(-1.0 * (self.H_delta) - 2.5 * self.Vy,-40,40)
        decalc =  k_P*( self.TanU  - self.u_input.Tan ) - k_d * self.u_input.Wz1 - k_i * self.Integral_H
        # Интеграл ошибки
        self.Integral_H += self.H_delta*self.dt
        # Ограничение -25 +25 обязательно
        self.delta_elerons = clamp(decalc,-25,25)

    def calc_PD(self,input:SAU_in,H_zad,V_zad,dt):
        self.u_input = input
        self.Vy = input.Vy
        self.dt = dt
        self.Vw = input.Vw
        self.H_delta = input.H - H_zad

        self.calc_drossel(V_zad/3.6)
        
        self.calc_delta_elerons()

        self.u_output.de = self.delta_elerons  # Заданное значение положения элеронов второго крыла
        self.u_output.P12_dr = self.drossel    # Заданное значение дросселя ВМГ 1-2
        self.u_output.P34_dr = self.drossel    # Заданное значение дросселя ВМГ 1-2

        self.Time += dt
        self.writeframe()

    def drop_u(self):
        return self.u_output