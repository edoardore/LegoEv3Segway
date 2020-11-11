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
motor=MoveTank(OUTPUT_A, OUTPUT_D)


gyro = GyroSensor(INPUT_4)

def initOffset(gyro):
    offset = 0
    i = 0
    t=time()
    while i < 100:
        offset += gyro.rate
        i += 1
    tt=time()-t
    offset = offset/t*3.1416/180
    print('Offset is '+str(offset), file=stderr)
    return offset

def sensor(Offset_gyro):
    angleDx = wheelDx.position+120
    angleSx = wheelSx.position+120
    ThetaM = (angleDx+angleSx)/2
    ThetaM = ThetaM*3.1416/180
    Psi_dot = gyro.rate
    Psi_dot = Psi_dot*3.1416/180
    Psi_dot-=Offset_gyro
    print('Sensor ThetaM is '+str(ThetaM) +
        '\nSensor psi_dot is '+str(Psi_dot), file=stderr)
    return (ThetaM, Psi_dot)

def integrate(List, Ti, Tf):
    i = Ti
    s = 0
    Ts = 0.05
    j = 0
    while i <= Tf and j < len(List):
        s += List[j]*Ts
        i += Ts
        j += 1
    return s

def derivativeDiscrete(L, x):
    #t=(tf-ti)/len(L)
    t=0.05
    if len(L)==1:
        return L[0]/(2*t)
    else:
        return (L[x+1] - L[x-1]) / (2*t)

def observer(Psi_dot, Theta, Offset_gyro):
    # Psi_dot -= Offset_gyro
    #Theta_dot = (wheelDx.speed+wheelSx.speed)/2
    #Theta_dot = Theta_dot*3.1416/180
    #print('Observer Theta is '+str(Theta) +
    #      '\nObserver Theta_dot is '+str(Theta_dot)+'\nObserver Psi_dot '+str(Psi_dot), file=stderr)
    #return(Theta, Theta_dot, Psi_dot)
    return(Psi_dot)
    
def control(Theta, Psi, Theta_dot, Psi_dot, Theta_int):
    K = [[-0.855, - 44.7896, - 0.9936, - 4.6061, -0.5000],
         [-0.855, - 44.7896, - 0.9936, - 4.6061, - 0.5000]]
    x = [[Theta], [Psi], [Theta_dot], [Psi_dot], [Theta_int]]
    u = [[0], [0]]
    for i in range(0, 2):
        j = 0
        for k in range(0, 5):
            u[i][j] += K[i][k]*x[k][j]
    print(u, file=stderr)
    return u

def rackUp(r):
    r.on_for_rotations(25, 0.30)

def rackDown(r):
    r.on_for_rotations(50, -0.30)

def main():
    offset = initOffset(gyro)
    sleep(1)
    motor.on_for_degrees(20, 20, -120, brake=True, block=True)
    rackUp(rack)
    wheelSx.stop()
    wheelDx.run_direct()
    wheelSx.stop()
    wheelSx.run_direct()
    u = [[]]
    ThetaList = []
    Psi_dotList=[]
    millisecondsStart = time()
    ThetaList.append(0)
    Psi_dotList.append(0)
    PI=3.1416
    count=0
    while True:
        t = time()
        if (t-millisecondsStart) > 10:
            break
        (ThetaM, Psi_dot) = sensor(offset)
        #(Theta, Theta_dot, Psi_dot) = observer(Psi_dot, Theta, offset)
        Psi_dotList.append(Psi_dot)
        Psi=integrate(Psi_dotList, millisecondsStart, t)
        print("Observer Psi is "+str(Psi), file=stderr)
        Theta=ThetaM+Psi
        if Theta<0:
            Theta%=(-2*PI)
        else:
            Theta%=(2*PI)
        print("Observer Theta is "+str(Theta), file=stderr)
        ThetaList.append(Theta)
        Theta_dot=derivativeDiscrete(ThetaList, len(ThetaList)-2)
        print("Observer Theta_dot is "+str(Theta_dot), file=stderr)
        Theta_int = integrate(ThetaList, millisecondsStart, t)
        print("Theta_int is "+str(Theta_int), file=stderr);
        u = control(Theta, Psi, Theta_dot, Psi_dot, Theta_int)
        count+=1
        Vr = u[1][0]
        Vr = Vr*40/8.2
        wheelDx.duty_cycle_sp = Vr
        wheelSx.duty_cycle_sp = Vr
        print(count, file=stderr)
    rackDown(rack)

main()