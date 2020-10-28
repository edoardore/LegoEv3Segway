#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
import time


# Initialize the EV3 brick.
ev3 = EV3Brick()
# Initialize ports
rack = Motor(Port.B)
wheelDx = Motor(Port.A)
wheelSx = Motor(Port.D)
gyro = GyroSensor(Port.S4)


def initOffset(gyro):
    values = []
    offset = 0
    i=0
    while i < 100:
        offset += gyro.speed()
        i+=1
    offset=offset/100
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


def observer(Psi_dot, Theta, Offset_gyro):
    Psi_dot -= Offset_gyro
    #Theta_dot = np.diff(Theta)
    #Psi = np.quad(Psi_dot)
    #Theta_int = np.quad(Theta)
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
    r.run_target(500, 90)


def rackDown(r):
    r.run_target(500, 0)


def main():
    offset = initOffset(gyro)
    print(offset)
    time.sleep(1)
    rackUp(rack)
    # TODO esegue modalità segway per 10 secondi
    rackDown(rack)


# Play a sound.
ev3.speaker.beep()
# Play another beep sound.
ev3.speaker.beep(600, 300)

main()
