from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair,
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from spike.operator import *
import hub
hube=PrimeHub()
motor_pair = MotorPair('F', 'B')
scanA = ColorSensor('A')
scanE = ColorSensor('E')

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def Curva(refer,power):#
    #Função cujo o objetivo é fazer uma curva com cálculo PIDem torno do eixo da roda que estará inerte, a partir de uma referência (medida em graus) .
    #No caso do operador decidir colocar mais ou menos potência no motor que executará o giro, a alteração poderá ser feita no segundo
    soma=0
    timer = Timer()
    timer.reset() #reseta timer
    hube.motion_sensor.reset_yaw_angle() # Reseta o angulo do giroscópio
    lastError = 0
    ref = refer
    tpCur = ((abs(ref)*140)/9000)
    kp = 3.3 #Define o valor do coeficiente de proporcional
    kd = 0 #Define o valor do coeficiente de derivativa
    ki=0
    while True:
        angle = hube.motion_sensor.get_yaw_angle()
        error = abs(ref) - abs(angle) #atribui o valor de erro a diferença entre o ref (a meta) e o angle (valor atual)
        P = kp * error # Realiza o cálculo de Proporcional
        soma += error
        I = ki * soma # Realiza o cálculo de Integral
        D = (error-lastError)*kd # Realiza o cálculo de Derivativa
        PID = P + I + D
        lastError = error

        if (ref>0):
            motor_pair.start_tank_at_power(int(PID+power), 0) # inicia o movimento com potência de PID no motor da esquerda
        else:
            motor_pair.start_tank_at_power(0, int(abs(PID)+abs(power))) # inicia o movimento com potência de PID
        if(abs(error)<1 or tpCur<timer.now()): #Se o erro for menor que 1 ou o temporizador for maior que o tempo mínimo para realizar a curva...
            break # o robô sai do loop
    motor_pair.stop() # e para de mover

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def Breaking(): #Não deixa o robô sair da programa
    motor_pair.stop()
    hube.right_button.wait_until_pressed()

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def Andar(condicao,ref):
    velocidade = 75
    motorB = Motor('B')
    motorF = Motor('F')
    motorB.set_degrees_counted(0)
    motorF.set_degrees_counted(0)
    hube.motion_sensor.reset_yaw_angle()
    BOOL =True
    if(condicao>0):
        kp = 4.6
        kd = 2.4
        ki = .00000007
    else:
        kp = 3
        kd = 0
        ki = 0.00001
    refer = ref
    lastError = 0
    vel = (velocidade*4)/9 #fração de 75, que será a velocidade inicial do robô até que ele atinja 75% de potência
    velComp = vel
    soma=0
    while True:
        angle = hube.motion_sensor.get_yaw_angle() # atribui o angulo lido a uma variável
        error = ref - angle
        P = kp * error
        soma += error
        I = ki * soma
        D = (error-lastError)*kd
        PID = P + I + D
        lastError = error
        if (condicao>0):
            motor_pair.start_at_power(int(vel), int(PID))
        else:
            motor_pair.start(int(-(PID)), int(-(vel)))
        if((abs(condicao)*100)<((abs(motorB.get_degrees_counted()))+abs(motorF.get_degrees_counted()))/4):
            break
        if ((velComp<velocidade)and(BOOL)):
            vel+=1
            velComp+=1
        if ((((abs(condicao)*100)*30)/100)> ((abs(condicao)*100)-((abs(motorB.get_degrees_counted()))+abs(motorF.get_degrees_counted()))/4)):
            BOOL = False
            if(abs(vel)>((velocidade*2)/6)):
                vel -= (1.8/abs(condicao))
            else:
                vel-=(1/abs(condicao))
                
                
        lastError = error
    motor_pair.stop()

#--------------------------------------------------------------------------------------------------------------------------------------------------------------
def DefensivoE():
 #Função com o objetivo de realizar a primeira rota do tapete, onde o robô leva os defensivos até as áreas de plantio, para os depósitos e para a base.
    Andar(2.3,0) #base area vermelha
    Curva(33,0)
    Curva(-33,0) #duas curvas são para ajeitar
    Andar(-2.1,0) #voltar até a base
    wait_for_seconds(.1)
    Curva(45,0) 
    Andar(1.85,0) #andar até o terceiro defensivo
    Curva(40,0)
    Andar(1.45,0)
    Curva(-75,0) #curva que coloca o defensivo no depósito
    Andar(.3, 0)
    Andar(-1.8,0)
    Curva(60,0)
    Andar(2.6,0)
    Andar(-1.2,0)
    Curva(33,0)
    Andar(-4,0)
    Curva(-50,0)
    Andar(-3.9,0)

def DefensivoD():
    Andar(2.5,0)
    Curva(-34,0)
    Curva(34,0)
    Andar(-2.5,0)
    wait_for_seconds(.1)
    Curva(-45,0)
    Andar(1.94,0)
    Curva(-50,0)
    Andar(1.39,0)
    Curva(85,0)
    Andar(.3,0)
    Andar(-1.4,0)
    Curva(-73,0)
    Andar(2.6,0)
    Andar(-1.2,0)
    Curva(-25,0)
    Andar(-4,0)
    Curva(50,0)
    Andar(-3.5,0)
#------------------------------------------------------------------------------------------------------------------------------------
Andar(40,0)
