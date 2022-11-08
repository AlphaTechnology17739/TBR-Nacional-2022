from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()
motor_pair = MotorPair('F', 'B')
scanA = ColorSensor('A')
scanE = ColorSensor('E')

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def Curva(refer,power):  #
    #Função cujo o objetivo é fazer uma curva com cálculo PID  em torno do eixo da roda que estará inerte, a partir de uma referência (medida em graus) .
    #No caso do operador decidir colocar mais ou menos potência no motor que executará o giro, a alteração poderá ser feita no segundo 
    
    timer = Timer()
    timer.reset() #reseta timer
    hub.motion_sensor.reset_yaw_angle() # Reseta o angulo do giroscópio
    lastError = 0
    ref = refer 
    tpCur = ((abs(ref)*140)/9000)
    kp = 2.3 #Define o valor do coeficiente de proporcional
    kd = .66 #Define o valor do coeficiente de derivativa
    if ref>0:
        ki = 0.00000022 #Define o valor do coeficiente de integral
    else:
        ki = -0.00000022 #Define o valor do coeficiente de integral
    while True:
        angle = hub.motion_sensor.get_yaw_angle() # atribui o angulo lido a uma variável
        error = ref - angle #atribui o valor de erro a diferença entre o ref (a meta) e o angle (valor atual)
        P = kp * error # Realiza o cálculo de Proporcional
        soma += error
        I = Ki * soma # Realiza o cálculo de Integral
        D = (error-lastError)*kd # Realiza o cálculo de Derivativa
        PID = P + I + D
        lastError = error

        if (ref>0):
            motor_pair.start_tank_at_power(PID+power, 0) # inicia o movimento com potência de PID no motor da esquerda
        else:
            motor_pair.start_tank_at_power(0, PID+power) # inicia o movimento com potência de PID
        if(abs(error)<1 or tpCur<timer.now()): #Se o erro for menor que 1 ou o temporizador for maior que o tempo mínimo para realizar a curva...
            break # o robô sai do loop
    motor_pair.stop() # e para de mover

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def Breaking(): #Não deixa o robô sair da programa
    br=True
    while br:
        if (hub.left_button.is.pressed() or hub.right_button.is.pressed()):
            br = !br

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def Andar(condicao,ref):
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

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def CheckAndGo(area):
    GoTo(area)
    encerra = True
    while (encerra):
        if ((scanA.get_color()=='red')or(scanE.get_color()=='red')):
            hub.status_light.on('red')
            encerra = False
            Vermelho(area)
        elif ((scanA.get_color()=='blue')or(scanE.get_color()=='blue')):
            hub.status_light.on('blue')
            encerra = False
            Azul(area)
        elif ((scanA.get_color()=='green')or(scanE.get_color()=='green')):
            hub.status_light.on('green')
            encerra = False
            Verde(area)
        elif ((scanA.get_color()=='yellow')or(scanE.get_color()=='yellow')):
            hub.status_light.on('yellow')
            encerra = False
            Amarelo(area)
        else:
            motor_pair.start_tank(20, 20)

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def GoTo(area):
    if(area==1):
        Curva(-70,0)
        Andar(.9,0)
        Curva(70,0)
        Andar(.24,0)
    elif(area==2):
        Curva(70,0)
        Andar(.67,0)
        Curva(-68,0)
        Andar(.25,0)
    elif(area==3):
        Curva(50,0)
        Andar(3.3,0)
        Curva(-76,0)
        Andar(1.75,0)
    elif(area==4):
        Curva(-50,0)
        Andar(3.2,0)
        Curva(76,0)
        Andar(1.65,0)

#--------------------------------------------------------------------------------------------------------------------------------------------------------------

def Vermelho(area):
    if(area==1):
        Curva(-170,0)
        Andar(0.8,0)
        Andar(-1.1,0)
        Curva(-73,0)
        Andar(3,0)
    elif(area==2):
        Curva(-115,0)
        Andar(3.3,0)
        Andar(-2.7,0)
        Curva(-75,0)
        Andar(1.5,0)
    elif(area==3):
        Andar(1.2,0)
        Curva(-55,0)
        Andar(2.7,0)
        Curva(-50,0)
        Andar(1,0)
        Curva(-50,0)
        Andar(3.8,0)
        Andar(-1.8,0)
        Curva(-50,0)
        Andar(4,0)
    elif(area==4):
        Curva(-100,0)
        Curva(-100,0)
        Andar(3.5,0)
        Andar(-1,0)
        Curva(-55,0)
        Andar(3.4,0)

def Amarelo(area):
    if(area==1):
        Curva(113,0)
        Andar(3.7,0)
        Andar(-2.7,0)
        Curva(75,0)
        Andar(1.5,0)
    elif(area==2):
        Curva(170,0)
        Andar(.8,0)
        Andar(-1.1,0)
        Curva(62,0)
        Andar(3.5,0)
    elif(area==3):
        Curva(100,0)
        Curva(100,0)
        Andar(3.5,0)
        Andar(-1,0)
        Curva(-52,0)
        Andar(3.4,0)
    elif(area==4):
         Andar(.8,0)
        Curva(55,0)
        Andar(2.5,0)
        Curva(60,0)
        Andar(1,0)
        Curva(30,0)
        Andar(3,0)
        Andar(-1.8,0)
        Curva(40,0)
        Andar(4,0)

def Verde(area):
    if(area==1):
        Curva(-20,0)
        Andar(4.7,0)
        Andar(-5.4,0)
        Curva(-30,0)
        Andar(-3,0)
    elif(area==2):
        Curva(25,0)
        Andar(4,0)
        Curva(-110,0)
        Andar(5.3)
        Curva(25,0)
        Andar(-0.8,0)
        Curva(-110,0)
        Andar(3.4,0)
        Curva(-45,0)
        Andar(4.7,0)
    elif(area==3):
        Andar(1.2,0)
        Curva(-55,0)
        Andar(3.5,0)
        Curva(25,0)
        Andar(-.8,0)
        Curva(-135)
        Andar(4,0)
    elif(area==4):
        Curva(-75,0)
        Andar(1.2,0)
        Andar(-.7,0)
        Curva(-135,0)
        Andar(2.3,0)
        Curva(-40,0)
        Andar(3.9,0)

def Azul(area):
    if(area==1):
        Curva(-20,0)
        Andar(4,0)
        Curva(110,0)
        Andar(4.8,0)
        Curva(-25,0)
        Andar(-.8,0)
        Curva(110,0)
        Andar(3.5,0)
        Curva(55,0)
        Andar(3.7,0)
    if(area==2):
        Curva(25,0)
        Andar(4.7,0)
        Andar(-4.7,0)
        Curva(20,0)
        Andar(-4,0)
    if(area==3):
        Curva(75,0)
        Andar(1.2,0)
        Andar(-.7,0)
        Curva(135,0)
        Andar(2.3,0)
        Curva(40,0)
        Andar(3.9,0)
     if(area==4):
        Andar(1,0)
        Curva(50,0)
        Andar(3.5,0)
        Andar(-.7,0)
        Curva(105,0)
        Andar(2.5,0)
        Curva(40,0)
        Andar(3.9,0)
#--------------------------------------------------------------------------------------------------------------------------------------

def DefensivoE():
 #Função com o objetivo de realizar a primeira rota do tapete, onde o robô leva os defensivos até as áreas de plantio, para os depósitos e para a base.  
    Andar(2.3,0)
    Curva(33,0)
    Curva(-33,0)
    Andar(-2.1,0)
    wait_for_seconds(.1)
    Curva(45,0)
    Andar(1.85,0)
    Curva(40,0)
    Andar(1.34,0)
    Curva(-85,0)
    Andar(-1.8,0)
    Curva(72,0)
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
def MudasPequenas ():
    Andar(3.6,0)
    Curva(-50,0)
    Andar(4,0)
    Andar(-8.5,-2)
    wait_for_seconds(.15)
    Andar(3.27,0)
    Curva(90,0)
    Andar(3.3,0)
    Andar(-8,0)
    Andar(.5,0)
    wait_for_seconds(.15)
    Curva(80,0)
    Andar(2.3,0)
