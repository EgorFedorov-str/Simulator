from UAV_update import SAU_in,SAU_out
from math import *
from pandas import DataFrame, concat
# Функция ограничения значения
clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

class SAU:
    def __init__(self):
        self.u_output = SAU_out()
        # Шаг интегрирования 
        self.dt = 0.1
        # Время моделирования
        self.Time = 0
        # Заданный угол тангажа
        self.TangU = 0
        # Производная заданного угла тангажа
        self.TangUDt = 0
        # Интеграл ошибки
        self.Integral_H = 0
        # Заданное значение положения дросселя для ВМГ
        self.P_reg = 0
        # Заданный угол отклонения руля высоты
        self.delta_lift = 0
        
        # Заданные регулируемые значения 
        # Вариант 1
        # Заданная высота, м.
        self.H_zad = 110
        # Заданная приборная скорость, км/ч
        self.Vpr_zad = 120


        # Шаблон записи для отладки
        self.writenames =list(["Time","TanU","TanU_dt","P_reg"])
        data_sau = [self.Time,self.TangU,self.TangUDt,self.P_reg]
        self.db = DataFrame([data_sau],columns = self.writenames)
    # Функция записи данных
    def writeframe(self):
        data_sau = [self.Time,self.TangU,self.TangUDt,self.P_reg]
        frame = DataFrame([data_sau],columns = self.writenames)
        self.db = concat([self.db, frame], ignore_index=True)
    
    # Шаблонная функция вычисления заданного положения дросселя для регулирования приборной скорости
    def calc_drossel(self):
        # Заданные значения
        # Заданная приборная скорость, км/ч
        Vpr_zad = self.Vpr_zad/3.6
        # Параметры которые могут понадобится для расчета 
        # Текущая приборная скорость, км/ч
        Vpr_now =  self.u_input.Vw/3.6
        # Расчет
        k_P = 0.5 # Пропорциональный коэффициент
        self.P_reg = 0.1 + k_P*(Vpr_zad - Vpr_now)
        # Ограничение [0.1,1] обязательно
        self.drossel = clamp(self.P_reg,0.1,1)
    # Шаблонная функция вычисления заданного отклонения руля высоты для регулирования высоты 
    def calc_delta_elerons(self):
        # Заданные значения
        H_zad = self.H_zad
        # Параметры которые могут понадобится для расчета 
        # Текущий угол тангажа, град.
        Tang = self.u_input.Tan
        # Текущая угловая скорость по оси ОZ, в связанной СК
        Wz1 = self.u_input.Wz1
        # Текущая высота БВС, м.
        H_now = self.u_input.H
        # Текущая вертикальная скорость БВС, м.
        Vy = self.u_input.Vy
        # Эталонный расчет

        # Пропорциональное звено
        K_beta = -2.7401
        K_gamma = 5.3728
        K_H = 0.2
        K_theta = 1.7039
        K_vy_1 = 5.7591
        K_vy_2 = 2.8941
        K_wz = 0.9
        k_P = 1.5
        # Интегральное звено
        k_i = 0.05
        # Дифференциальное звено
        k_d = 0.5 
        #0.01

        # Пропорциональное звено регулятора
        dv = (Tang + (H_zad - H_now) * K_H * K_vy_1 - Vy * K_vy_2) * K_theta + Wz1 * K_wz

        
        # Заданный угол тангажа в зависимости от высоты, с ограничением
        self.TangU = clamp(-1.0 * (H_now - H_zad) - 0.5 * Vy,-40,40)
        delta_elerons_calc = k_P*( self.TangU  - Tang ) - k_d * Wz1 - k_i * self.Integral_H
        # Интеграл ошибки
        self.Integral_H += (H_now-H_zad)*self.dt
        # Ограничение -25 +25 обязательно
        self.delta_elerons = clamp(delta_elerons_calc,-25,25)
    
    # Основная функция расчета 
    def calc_PID(self,input:SAU_in):
        self.u_input = input
        
        self.calc_drossel()
        self.calc_delta_elerons()


        self.u_output.de = self.delta_elerons  # Заданное значение положения элеронов второго крыла
        self.u_output.P12_dr = self.drossel    # Заданное значение дросселя ВМГ 1-2
        self.u_output.P34_dr = self.drossel    # Заданное значение дросселя ВМГ 1-2
        self.Time += self.dt
        self.writeframe()
        return self.u_output

    def drop_u(self):
        return self.u_output
