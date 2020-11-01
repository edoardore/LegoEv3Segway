#!/usr/bin/env python3

from ev3dev2.led import Leds
from ev3dev2.motor import Motor, MoveTank, OUTPUT_A, OUTPUT_D, OUTPUT_B
from ev3dev2.sound import Sound
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import GyroSensor
import time



# Initialize ports
rack = Motor(OUTPUT_B)
wheelDx = Motor(OUTPUT_A)
wheelSx = Motor(OUTPUT_D)
motor = MoveTank(OUTPUT_A, OUTPUT_D)  # Drive using two motors (tank mode)
gyro = GyroSensor(INPUT_4)


def initOffset(gyro):
    offset = 0
    i = 0
    while i < 100:
        offset += gyro.speed()
        i += 1
    offset = offset/100
    return offset


# sensors model (rad/s)
def sensor(Offset_gyro):
    angleDx = wheelDx.angle()
    angleSx = wheelSx.angle()
    Theta = (angleDx+angleSx)/2
    Theta = Theta*3.14159265358979323846/180
    Psi_dot = gyro.speed()+Offset_gyro
    Psi_dot = Psi_dot*3.14159265358979323846/180
    return (Theta, Psi_dot)

def integrate(ThetaList, Ti, Tf, Ts=0.1):
    i = Ti
    s = 0
    while i <= Tf:
        s += ThetaList[i]*Ts
        i += Ts
    return s

def observer(Psi_dot, Theta, Offset_gyro):
    Psi_dot -= Offset_gyro
    Theta_dot = (wheelDx.speed()+wheelSx.speed())/2
    Theta_dot=Theta_dot*3.14159265358979323846/180
    Psi = gyro.angle()-Offset_gyro
    return(Theta, Psi, Theta_dot, Psi_dot, Theta_int)


def control(Theta, Psi, Theta_dot, Psi_dot, Theta_int):
    K = [[-0.855, - 44.7896, - 0.9936, - 4.6061, -0.5000],
         [-0.855, - 44.7896, - 0.9936, - 4.6061, - 0.5000]]
    print(K)
    x = [Theta, Psi, Theta_dot, Psi_dot, Theta_int]
    u = [[]]
    for i in range(len(K)):
        for h in range(len(x)):
            j = 1
            u[i][j] = -K[i][h]*x[h]
    return u


def rackUp(r):
    r.run_target(900, 90)


def rackDown(r):
    r.run_target(900, 0)


def run():
    offset=initOffset(gyro)
    print(offset)
    time.sleep(1)
    rackUp(rack)
    u=[[]]
    ThetaList=[]
    millisecondsStart = int(round(time.time() * 1000))
    # TODO esegue modalità segway per 10 secondi
    while true:
        time=int(round(time.time() * 1000))-millisecondsStart
        print(time)
        if time>10000:
            break
        (Theta, Psi_dot)=sensor(offset)
        (Theta, Psi, Theta_dot, Psi_dot) = observer(Theta, Psi_dot, offset)    
        ThetaList.append(Theta)  
        Theta_int=integrate(ThetaList, millisecondsStart, time)  
        u = control(Theta, Psi, Theta_dot, Psi_dot, Theta_int)
        #TODO & MOVE TANK
        motor.run_angle(u[2], u[0], then=Stop.HOLD, wait=True)
    rackDown(rack)
    # Play a sound.
    ev3.speaker.beep()
    # Play another beep sound.
    ev3.speaker.beep(600, 300)

motor.on(50, 50)
