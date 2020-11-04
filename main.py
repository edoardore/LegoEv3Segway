#!/usr/bin/env python3
from ev3dev2.led import Leds
from ev3dev2.motor import Motor, MoveTank, OUTPUT_A, OUTPUT_D, OUTPUT_B, SpeedDPS
from ev3dev2.sound import Sound
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import GyroSensor
from sys import stderr
from time import sleep, time
import threading


rack = Motor(OUTPUT_B)
wheelDx = Motor(OUTPUT_A)
wheelSx = Motor(OUTPUT_D)
motor = MoveTank(OUTPUT_A, OUTPUT_D)
gyro = GyroSensor(INPUT_4)


class Controllo(threading.Thread):
    def __init__(self, m):
        threading.Thread.__init__(self)
        self.m = m

    def initOffset(self, gyro):
        offset = 0
        i = 0
        while i < 100:
            offset += gyro.rate
            i += 1
        offset = offset/100*3.14159265358979323846/180
        print('Offset is '+str(offset), file=stderr)
        return offset

    def sensor(self, Offset_gyro):
        angleDx = wheelDx.position
        angleSx = wheelSx.position
        Theta = (angleDx+angleSx)/2
        Theta = Theta*3.14159265358979323846/180
        Psi_dot = gyro.rate+Offset_gyro
        Psi_dot = Psi_dot*3.14159265358979323846/180
        print('Sensor Theta is '+str(Theta) +
              '\nSensor psi_dot is '+str(Psi_dot), file=stderr)
        return (Theta, Psi_dot)

    def integrate(self, ThetaList, Ti, Tf):
        i = Ti
        s = 0
        Ts = 0.1
        j = 0
        while i <= Tf and j < len(ThetaList):
            s += ThetaList[j]*Ts
            i += Ts
            j += 1
        print('Integrate is '+str(s), file=stderr)
        return s

    def observer(self, Psi_dot, Theta, Offset_gyro):
        Psi_dot -= Offset_gyro
        Theta_dot = (wheelDx.speed+wheelSx.speed)/2
        Theta_dot = Theta_dot*3.14159265358979323846/180
        (Psi, rate) = gyro.angle_and_rate
        Psi = Psi*3.14159265358979323846/180
        print('Observer Theta is '+str(Theta)+'\nObserver Psi is '+str(Psi) +
              '\nObserver Theta_dot is '+str(Theta_dot)+'\nObserver Psi_dot '+str(Psi_dot), file=stderr)
        return(Theta, Psi, Theta_dot, Psi_dot)

    def control(self, Theta, Psi, Theta_dot, Psi_dot, Theta_int):
        K = [[-0.855, - 44.7896, - 0.9936, - 4.6061, -0.5000],
             [-0.855, - 44.7896, - 0.9936, - 4.6061, - 0.5000]]
        x = [[Theta], [Psi], [Theta_dot], [Psi_dot], [Theta_int]]
        u = [[0], [0]]
        for i in range(0, 2):
            j = 0
            for k in range(0, 5):
                u[i][j] += K[i][k]*x[k][j]
            u[i][j] = -u[i][j]
        print(u, file=stderr)
        return u

    def rackUp(self, r):
        r.on_for_rotations(25, 0.30)

    def rackDown(self, r):
        r.on_for_rotations(50, -0.30)

    def run(self):
        offset = self.initOffset(gyro)
        sleep(1)
        motor.on_for_degrees(45, 45, -95, brake=True, block=True)
        self.rackUp(rack)
        u = [[]]
        ThetaList = []
        millisecondsStart = time()
        while True:
            t = time()
            if (t-millisecondsStart) > 10:
                break
            (Theta, Psi_dot) = self.sensor(offset)
            (Theta, Psi, Theta_dot, Psi_dot) = self.observer(
                Psi_dot, Theta, offset)
            ThetaList.append(Theta)
            Theta_int = self.integrate(ThetaList, millisecondsStart, t)
            u = self.control(Theta, Psi, Theta_dot, Psi_dot, Theta_int)
            self.m.update(u)
        self.rackDown(rack)


class Motore:
    def update(self, u):
        speed = abs(u[1][0])*180/3.14
        if speed > 1050:
            speed = 1050
        motor.on_for_seconds(SpeedDPS(speed), SpeedDPS(speed), 0.1, brake=False, block=False)


motore = Motore()
controllo = Controllo(motore)
controllo.start()
