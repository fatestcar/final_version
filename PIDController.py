#!/usr/bin/env python
# -*- coding: utf-8 -*-

import ctypes

class PIDController:
    def __init__(self):
        ll = ctypes.cdll.LoadLibrary
        self.lib = ll("./test.so")
        self.d_max = 15.0  # 前轮中心在中轴线右侧的最大距离 单位：cm
        self.v = 5.0  # 恒定速度  单位：cm/s
        self.l_max = 45.0
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.1
        self.currentError = 0.0  # 当前时刻偏差
        self.lastError = 0.0  # 上一时刻偏差
        self.sigmaError = 0.0  # 累计偏差

    def getOutput(self, x):
        if x < (-1) * self.d_max:
            # 若超过左侧距离最大值，则记为最大值
            x = (-1) * self.d_max
        elif x > self.d_max:
            # 若超过右侧距离最大值，则记为最大值
            x = self.d_max
        # 将PID算法对于距离的调整输出转化为对于转动角度的输出
        res = x * (self.l_max / self.d_max)
        return res

    def controll(self, pos):
        error = 0 - pos
        self.lastError = self.currentError
        self.currentError = error
        self.sigmaError = self.sigmaError + error
        ux = self.kp * error + self.ki * self.sigmaError + self.kd * (self.currentError - self.lastError)
        output = self.getOutput(ux)

        # 在这里输出转动角度
        self.lib.turnTo(int(output))
        self.lib.delay(1000)
        return output
