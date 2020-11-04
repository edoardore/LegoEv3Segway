#!/usr/bin/env python3
from ev3dev2.led import Leds
from ev3dev2.motor import Motor, MoveTank, OUTPUT_A, OUTPUT_D, OUTPUT_B
from ev3dev2.sound import Sound
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import GyroSensor
from sys import stderr
from time import sleep, time


# Initialize ports
rack = Motor(OUTPUT_B)
wheelDx = Motor(OUTPUT_A)
wheelSx = Motor(OUTPUT_D)
motor = MoveTank(OUTPUT_A, OUTPUT_D)  # Drive using two motors (tank mode)
gyro = GyroSensor(INPUT_4)

# returns deg/sec


def initOffset(gyro):
    offset = 0
    i = 0
    while i < 100:
        offset += gyro.rate
        i += 1
    offset = offset/100
    return offset

# sensors model (rad/s)


def sensor(Offset_gyro):
    angleDx = wheelDx.position
    angleSx = wheelSx.position
    Theta = (angleDx+angleSx)/2
    Theta = Theta*3.14159265358979323846/180
    Psi_dot = gyro.rate+Offset_gyro
    Psi_dot = Psi_dot*3.14159265358979323846/180
    return (Theta, Psi_dot)


def integrate(ThetaList, Ti, Tf):
    i = Ti
    s = 0
    Ts=Tf/len(ThetaList)
    while i <= Tf:
        s += ThetaList[i]*Ts
        i += Ts
    return s


def observer(Psi_dot, Theta, Offset_gyro):
    Psi_dot -= Offset_gyro
    Theta_dot = (wheelDx.speed+wheelSx.speed)/2
    Theta_dot = Theta_dot*3.14159265358979323846/180
    Psi = gyro.angle
    return(Theta, Psi, Theta_dot, Psi_dot)


def control(Theta, Psi, Theta_dot, Psi_dot, Theta_int):
    K = [[-0.855, - 44.7896, - 0.9936, - 4.6061, -0.5000],
         [-0.855, - 44.7896, - 0.9936, - 4.6061, - 0.5000]]
    x = [[Theta], [Psi], [Theta_dot], [Psi_dot], [Theta_int]]
    u = [[0], [0]]
    for i in range(0, 2):
        j = 0
        for k in range(0, 5):
            u[i][j] += K[i][k]*x[k][j]
        u[i][j] = -u[i][j]
    return u


def rackUp(r):
    r.on_for_rotations(25, 0.30)


def rackDown(r):
    r.on_for_rotations(50, -0.30)


offset = initOffset(gyro)
print(offset, file=stderr)
sleep(1)
rackUp(rack)
u = [[]]
ThetaList = []
millisecondsStart = time()
# TODO esegue modalità segway per 10 secondi
while True:
    t = time()-millisecondsStart
    if t > 10:
        break
    (Theta, Psi_dot) = sensor(offset)
    (Theta, Psi, Theta_dot, Psi_dot) = observer(Theta, Psi_dot, offset)
    ThetaList.append(Theta)
    Theta_int = integrate(ThetaList, millisecondsStart, t)
    u = control(Theta, Psi, Theta_dot, Psi_dot, Theta_int)
    print (u, file =stderr)
    motor.on_for_degrees(u[1][0]/9, u[1][0]/9, u[0][0], brake=True, block=True)
rackDown(rack)