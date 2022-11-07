from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()
motor_pair = MotorPair('F', 'B')

#-------------------------------------------------------------------------------

def curva(refer,power):
    timer = Timer()
    timer.reset()
    hub.motion_sensor.reset_yaw_angle() # Reseta o angulo do giroscópio
    lastError = 0
    ref = refer
    tpCur = ((abs(ref)*140)/9000)
    kp = 2.3
    kd = .66
    if ref>0:
        ki = 0.00000022
    else:
        ki = -0.00000022
    while True:
        angle = hub.motion_sensor.get_yaw_angle() # atribui o angulo lido a uma variável
        error = ref - angle
        P = kp * error
        soma += error
        I = Ki * soma
        D = (error-lastError)*kd
        PID = P + I + D
        lastError = error

        if (ref>0):
            motor_pair.start_tank_at_power(PID+power, 0)
        else:
             motor_pair.start_tank_at_power(0, PID+power)
        if(abs(error)<1 or tpCur<timer.now()):
            break
    motor_pair.stop()

#-------------------------------------------------------------------------------

def breaking():
    br=True
    while br:
        if (hub.left_button.is.pressed() or hub.right_button.is.pressed()):
            br = !br

#-------------------------------------------------------------------------------

def andar(condicao,ref):
    motorB = hub.port.B.motor
    motorF = hub.port.F.motor
    motorB.set_degrees_counted(0)
    motorF.set_degrees_counted(0)
    hub.motion_sensor.reset_yaw_angle()
    BOOL =True
    kp = 4.6
    kd = 2.4
    ki = .00000007
    refer = ref
    lastError = 0
    vel = (70*4)/9 #fração de 70, que será a velocidade inicial do robô até que ele atinja 70% de potência 
    velComp = vel
    while True:
        angle = hub.motion_sensor.get_yaw_angle() # atribui o angulo lido a uma variável
        error = ref - angle
        P = kp * error
        soma += error
        I = Ki * soma
        D = (error-lastError)*kd
        PID = P + I + D
        lastError = error
        if (condicao>0):
            motor_pair.start(PID, vel)
        else:
            motor_pair.start(-(PID), -(vel))
        if((abs(condicao)*100)<((abs(motorB.get_degrees_counted()))+abs(motorF.get_degrees_counted()))/4):
            break
        if ((velComp<70)and(BOOL)):
            vel+=2
            velComp+=2
        if ((((abs(condicao)*100)*30)/100)> ((abs(condicao)*100)-((abs(motorB.get_degrees_counted()))+abs(motorF.get_degrees_counted()))/4)):
            if(vel>((70*2)/6)):
                if(0<condicao):
                    BOOL = False
                    vel -= (9/condicao)
                else: 
                    BOOL = False
                    vel += (9/condicao)
        lastError = error
    motor_pair.stop()
