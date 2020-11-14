#!/usr/bin/env python3
from ev3dev2.led import Leds
from ev3dev2.motor import Motor, OUTPUT_A, OUTPUT_D, OUTPUT_B, MoveTank
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import GyroSensor
from sys import stderr
from time import sleep, time


rack = Motor(OUTPUT_B)
wheelDx = Motor(OUTPUT_A)
wheelSx = Motor(OUTPUT_D)
motor = MoveTank(OUTPUT_A, OUTPUT_D)
gyro = GyroSensor(INPUT_4)
PI = 3.1416


def getGyroRate(readings):
    mean = 0
    for i in range(1, readings):
        mean = mean + gyro.rate
    mean = mean / readings
    return mean


def calibrate_gyro(loops):
    mean = 0
    for i in range(1, loops):
        mean = mean + getGyroRate(5)
    mean = mean / loops
    return mean


def sensor(Offset_gyro, ThetaTimeList, Psi_dotTimeList):
    angleDx = wheelDx.position
    angleSx = wheelSx.position
    t = time()
    ThetaTimeList.append(t)
    ThetaM = (angleDx+angleSx)/2
    ThetaM = ThetaM*PI/180
    Psi_dot = gyro.rate
    t = time()
    Psi_dotTimeList.append(t)
    Psi_dot = Psi_dot*PI/180
    Psi_dot -= Offset_gyro
    '''print('Sensor ThetaM is '+str(ThetaM) +
         '\nSensor psi_dot is '+str(Psi_dot), file=stderr)
    '''
    return (ThetaM, Psi_dot)


def integrate(List, TimeList):
    s = 0
    j = 1
    while j < len(List):
        Ts = TimeList[j]-TimeList[j-1]
        s += List[j]*Ts
        j += 1
    return s


def derivativeDiscrete(L, x, TimeList):
    t = TimeList[x]-TimeList[x-1]
    return (L[x] - L[x-1]) / t


def control(Theta, Psi, Theta_dot, Psi_dot, Theta_int):
    K = [[-0.8559, - 44.7896, - 0.9936, - 4.6061, -0.5000],
         [-0.8559, - 44.7896, - 0.9936, - 4.6061, - 0.5000]]
    x = [[Theta], [Psi], [Theta_dot], [Psi_dot], [Theta_int]]
    u = [[0], [0]]
    for i in range(0, 2):
        j = 0
        for k in range(0, 5):
            u[i][j] += K[i][k]*x[k][j]
    #print(u, file=stderr)
    return u


def control2(Theta, Psi, Theta_dot, Psi_dot, Theta_int):
    u = (-0.8559*Theta)+(-44.7896*Psi)+(-0.9936*Theta_dot)+(-4.6061*Psi_dot)+(-0.500*Theta_int)


def rackUp(r):
    r.run_to_rel_pos(position_sp=90, speed_sp=500, stop_action="brake")


def rackDown(r):
    r.run_to_rel_pos(position_sp=-90, speed_sp=500, stop_action="brake")


def main():
    offset = calibrate_gyro(5)*PI/180
    print("Offset is "+str(offset), file=stderr)
    sleep(1)
    wheelSx.stop()
    wheelDx.run_direct()
    wheelSx.stop()
    wheelSx.run_direct()
    u = [[]]
    ThetaList = []
    Psi_dotList = []
    ThetaTimeList = []
    Psi_dotTimeList = []
    count = 0
    millisecondsStart = time()
    while True:
        t = time()
        if (t-millisecondsStart) > 10:
            break
        if count == 0:
            Theta = 0
            Psi = -8.7*PI/180
            Theta_dot = 0
            Psi_dot = 0
            Theta_int = 0
            ThetaList.append(0)
            Psi_dotList.append(0)
            t1 = time()
            ThetaTimeList.append(t1)
            Psi_dotTimeList.append(t1)
        u = control(Theta, Psi, Theta_dot, Psi_dot, Theta_int)
        count += 1
        Vr = u[1][0]
        Vr = Vr*100/9
        if Vr > 100:
            Vr = 100
        if Vr < -100:
            Vr = -100
        wheelDx.duty_cycle_sp = Vr
        wheelSx.duty_cycle_sp = Vr
        if count == 1:
            rackUp(rack)
        (ThetaM, Psi_dot) = sensor(offset, ThetaTimeList, Psi_dotTimeList)
        Psi_dotList.append(Psi_dot)
        Psi = integrate(Psi_dotList, Psi_dotTimeList)
        #print("Observer Psi is "+str(Psi), file=stderr)
        Theta = ThetaM+Psi
        #print("Observer Theta is "+str(Theta), file=stderr)
        ThetaList.append(Theta)
        Theta_dot = derivativeDiscrete(ThetaList, len(ThetaList)-1, ThetaTimeList)
        #print("Observer Theta_dot is "+str(Theta_dot), file=stderr)
        Theta_int = integrate(ThetaList, ThetaTimeList)
        #print("Theta_int is "+str(Theta_int), file=stderr)
    print(count, file=stderr)
    wheelSx.stop()
    wheelDx.stop()
    rackDown(rack)


main()