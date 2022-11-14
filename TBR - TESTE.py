from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair,
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import hub

hube=PrimeHub()
motor_pair = MotorPair('F', 'B')
scanA = ColorSensor('A')
scanE = ColorSensor('E')
luz = hube.light_matrix

# =========================================================================================================

# ==== CRIAÇÃO DE PIDs ====

# ---- PID CURVA ----
# Função cujo o objetivo é fazer uma curva com cálculo PID em torno do eixo da roda que estará inerte, a partir de uma referência (medida em graus) .
# No caso do operador decidir colocar mais ou menos potência no motor que executará o giro, a alteração poderá ser feita no segundo

def Curva(refer,power):
    if (hube.left_button.is_pressed()): # Verifica se o usuário não pretende sair do programa
        controle=True

    timer = Timer()
    timer.reset() #reseta timer
    hube.motion_sensor.reset_yaw_angle() # Reseta o angulo do giroscópio
    lastError = 0
    ref = refer
    error = 2
    soma = 0
    tpCur = ((abs(ref)*140)/9000)
    kp = 2.3 # Define o valor do coeficiente de proporcional
    kd = .66 # Define o valor do coeficiente de derivativa
    if ref>0:
        ki = 0.00000022 # Define o valor do coeficiente de integral
    else:
        ki = -0.00000022 # Define o valor do coeficiente de integral
    while True:
        '''if(abs(error)<1 or tpCur<timer.now() or controle): # Se o erro for menor que 1 <ou> o temporizador for maior que o tempo mínimo para realizar a curva <ou> a váriavel de controle for igual a 0...
            break # O robô sai do loop'''
        if (hube.left_button.is_pressed()):# Verifica se o usuário não pretende sair do programa
            controle=True
        angle = hube.motion_sensor.get_yaw_angle() # Atribui o angulo lido a uma variável
        error = (abs(ref) - abs(angle)) # Atribui o valor de erro a diferença entre o ref (a meta) e o angle (valor atual)
        P = kp * error # Realiza o cálculo de Proporcional
        soma += error
        I = ki * soma # Realiza o cálculo de Integral
        D = (error-lastError)*kd # Realiza o cálculo de Derivativa
        PID = P + I + D
        lastError = error

        if (ref>0):
            motor_pair.start_tank_at_power(int(PID+power), 0) # inicia o movimento com potência de PID no motor da esquerda
        else:
            motor_pair.start_tank_at_power(0, int(PID+power)) # inicia o movimento com potência de PID
    motor_pair.stop() # e para de mover

# ----------------------------------------------------------------------------------------------------------

# ---- PID ANDAR ----
# Função cujo o objetivo é fazer andar reto com cálculo PID, a partir de uma referência (medida em graus).
# O cálculo é aplicado na função de direção do robô.

def Andar(condicao,ref):

    if (hube.left_button.is_pressed()):# Verifica se o usuário não pretende sair do programa
        controle=True

    motorB = Motor('B')
    motorF = Motor('F')
    motorB.set_degrees_counted(0)
    motorF.set_degrees_counted(0)
    hube.motion_sensor.reset_yaw_angle()
    BOOL =True
    kp = 4.6
    kd = 2.4
    ki = .00000007
    refer = ref
    lastError = 0
    vel = (70*4)/9 # Fração de 70, que será a velocidade inicial do robô até que ele atinja 70% de potência
    velComp = vel
    soma=0
    while True:
        if(((abs(condicao)*100)<((abs(motorB.get_degrees_counted()))+abs(motorF.get_degrees_counted()))/4)    or controle):
            break
        if (hube.left_button.is_pressed()):# Verifica se o usuário não pretende sair do programa
            controle=True
        angle = hube.motion_sensor.get_yaw_angle() # Atribui o angulo lido a uma variável
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
            motor_pair.start(-(PID), -(vel))

        if ((velComp<70)and(BOOL)):
            vel+=2
            velComp+=2
        if ((((abs(condicao)*100)*30)/100)> ((abs(condicao)*100)-((abs(motorB.get_degrees_counted()))+abs(motorF.get_degrees_counted()))/4)):
            if(vel>((70*2)/6)):
                if(0<condicao):
                    BOOL = False
                    vel -= (7/condicao)
                else:
                    BOOL = False
                    vel += (7/condicao)
        lastError = error
    motor_pair.stop()

# =========================================================================================================

# ==== Criação de parada temporária até que algum evento aconteça ====

def Breaking():
    if (hube.left_button.is_pressed()):# Verifica se o usuário não pretende sair do programa
        controle=True
    if (not controle):
        motor_pair.stop()
        if (hube.left_button.is_pressed()):
            controle=True
        hube.right_button.wait_until_pressed() # Espera o botão ser apertado para sair da programação

# =========================================================================================================

# ==== Funções da primeira rota ====

# ---- primeira parte ----
# Função com o objetivo de realizar a primeira rota do tapete, onde o robô leva os defensivos
# até as áreas de plantio vermelha e verde e para o depósito a esquerda do tapete.
def DefensivoE():

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

# ----------------------------------------------------------------------------------------------------------

# ---- segunda parte ----
# Função com o objetivo de realizar a segunda rota do tapete, onde o robô leva os defensivos
# até as áreas de plantio amarela, verde e para o depósito a direita e para a base.

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

# =========================================================================================================

# ==== Funções da segunda rota ====

# ---- Chegada nas árvores ----
# Percorre um caminho (definido pelo programador entre 1 e 4) que leva a uma das árvores grandes.
# Ao terminar o caminho, o robô identifica a cor da árvore e é encaminhado para a função de nome igual a respectiva cor.

def CheckAndGo(area):
    GoTo(area)
    encerra = True
    while (encerra):
        if (hube.left_button.is_pressed()):
            controle=True
        if (controle):
            break

        if ((scanA.get_color()=='red')or(scanE.get_color()=='red')):
            hube.status_light.on('red')
            encerra = False
            Vermelho(area)
        elif ((scanA.get_color()=='blue')or(scanE.get_color()=='blue')):
            hube.status_light.on('blue')
            encerra = False
            Azul(area)
        elif ((scanA.get_color()=='green')or(scanE.get_color()=='green')):
            hube.status_light.on('green')
            encerra = False
            Verde(area)
        elif ((scanA.get_color()=='yellow')or(scanE.get_color()=='yellow')):
            hube.status_light.on('yellow')
            encerra = False
            Amarelo(area)
        else:
            motor_pair.start_tank(20, 20)

# ----------------------------------------------------------------------------------------------------------

# ---- chegada nas mudas ----
# Percorre o tapete até chegar em uma das árvores grandes.
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

# ----------------------------------------------------------------------------------------------------------

# ---- Possibilidades vermelho ----
# Após a checagem da cor da árvore como vermelha, o robô, dependendo de onde ele está,
# realiza uma programação que leva a árvore grande até a área da sua cor e volta pra base
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

# ---- Possibilidades amarelo ----
# Após a checagem da cor da árvore como amarela, o robô, dependendo de onde ele está,
# realiza uma programação que leva a árvore grande até a área da sua cor e volta pra base

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

# ---- Possibilidades verde ----
# Após a checagem da cor da árvore como verde, o robô, dependendo de onde ele está,
# realiza uma programação que leva a árvore grande até a área da sua cor e volta pra base

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

# ---- Possibilidades azul ----
# Após a checagem da cor da árvore como azul, o robô, dependendo de onde ele está,
# realiza uma programação que leva a árvore grande até a área da sua cor e volta pra base

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

# =========================================================================================================

# ==== Função da segunda rota ====
# Leva as mudas pequenas para as áreas da sua cor

def MudasPequenas():
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

# =========================================================================================================

# ==== Funções de layout ====

# ---- Play sub-function ----
# A função executa uma das rotas possíveis de acordo com a sub-programação que foi aberta
def Play(num_prog):
    if(num_prog==0): #Defensivos
        DefensivoE()
        Breaking()
        DefensivoD()
    elif(num_prog==1): #Mudas grandes
        for i in range(4):
            CheckAndGo(i+1)
            Breaking()

    elif(num_prog==2):#Mudas pequenas
        MudasPequenas()

# ----------------------------------------------------------------------------------------------------------

# ---- Layout ----
# altera o que o programa exibe dependendo da sub-programação que está selecionada

def Layout():
    global program
    if(program==0):
        hube.light_matrix.set_pixel(2,0)
        hube.light_matrix.set_pixel(2,4)
        for y in range(5):
            hube.light_matrix.set_pixel(1, (y))
            hube.light_matrix.set_pixel(3,(y))
    elif(program==1):
        
        hube.light_matrix.set_pixel(1,1)
        hube.light_matrix.set_pixel(1,4)
        hube.light_matrix.set_pixel(3,4)
        for y in range(5):
            hube.light_matrix.set_pixel(2, (y))
    elif(program==2):
        for y in range(3):
            for x in range(3):
                luz.set_pixel((x+1),(y*2))
        luz.set_pixel(3,1)
        luz.set_pixel(1,3)

# ----------------------------------------------------------------------------------------------------------

# ---- Rodando ----
# segunda programação a ser rodada, mas é a que permite a execução em loop do código e
# da autonomia ao usuário para realizar as decisões de sub-programação

def Rodando():
    global program

    Layout()

    if (hube.left_button.is_pressed()):
        controle=False
        luz.off()
        Play(program)

    elif (hube.right_button.is_pressed()):
        timer=Timer()
        hube.right_button.wait_until_released()
        tempoNow=timer.now()
        print(tempoNow)
        if (tempoNow>0):
            program-=1
            luz.off()
        else:
            program+=1
            luz.off()
    
# ----------------------------------------------------------------------------------------------------------

# ---- Main ----
# inicia o programa e faz um Setup inicial
def Main():
    
    while True:
        Rodando()
# ----------------------------------------------------------------------------------------------------------

#RODANDO PROGRAMAÇÃO PRINCIPAL:
program = 0
Main()
