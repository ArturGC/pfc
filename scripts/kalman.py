#! /usr/bin/env python3
import os
import rospy
import threading
import csv
import numpy as np
from std_msgs.msg import Float32MultiArray


## Variáveis Globais ##
lock = threading.Lock()

# Dados do sistema
dt = 0.05
dtIntervalo = dt/2
A = np.matrix([[1, dt],[0, 1]])
B = np.matrix([[0.5*dt*dt],[dt]])
H = np.matrix([[1, 0],[0, 1]])

# Dados do filtro de Kalman
t = None
Xcorrigido = np.matrix([[0],[0]])
desvioAcelerometro = 2.5
desvioEncoder = 0.15
desvioGPS = 0.55

P = np.matrix([[desvioAcelerometro*dt*dt*0.5, 0],[0, desvioAcelerometro*dt]])
Pcorrigido = P

R = np.matrix([[desvioGPS, 0],[0, desvioEncoder]])
Rv = np.matrix([[0, 0],[0, desvioEncoder]])
Rgps = np.matrix([[desvioGPS, 0],[0, 0]])


# Dados gerados durante execução
dadosAcelerometro = np.matrix([[],[],[],[]])
dadosVelocidade = np.matrix([[],[]])
dadosGPS = np.matrix([[],[],[],[]])
posicaoZero = np.matrix([['a'],[0],[0]])

dadosKalman = [[],[],[]]
aceleracaoDadosKalman = [[],[]]
velocidadeDadosKalman = [[],[]]
posicaoDadosKalman =[[],[]]


def desligandoNode():
    global dadosKalman
    global aceleracaoDadosKalman, velocidadeDadosKalman, posicaoDadosKalman

    diretorio = rospy.get_param('/dadosExecucao')[0]
    
    fileName = '{}/dadosKalman.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in dadosKalman:
            writer.writerow(lista)

    fileName = '{}/aceleracaoDadosKalman.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in aceleracaoDadosKalman:
            writer.writerow(lista)

    fileName = '{}/velocidadeDadosKalman.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in velocidadeDadosKalman:
            writer.writerow(lista)

    fileName = '{}/posicaoDadosKalman.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in posicaoDadosKalman:
            writer.writerow(lista)


def acelerometroMedidoCB(dados):
    global dadosAcelerometro, lock

    novosDados = np.matrix([[dados.data[0]],[dados.data[1]],[dados.data[2]],[dados.data[3]]])
    lock.acquire()
    dadosAcelerometro = np.concatenate([dadosAcelerometro, novosDados], 1)
    lock.release()


def velocidadeMedidaCB(dados):
    global dadosVelocidade, lock

    novosDados = np.matrix([[dados.data[0]],[dados.data[1]]])
    lock.acquire()
    dadosVelocidade = np.concatenate([dadosVelocidade, novosDados], 1)
    lock.release()


def gpsMedidoCB(dados):
    global dadosGPS, posicaoZero, lock

    if posicaoZero[0,0] == 'a':
        posicaoZero = np.matrix([[dados.data[0]],[dados.data[1]],[dados.data[2]]])

    posicaoAtual = np.matrix([[dados.data[0]],[dados.data[1]],[dados.data[2]]])
    deltaPosicao = abs(np.subtract(posicaoAtual, posicaoZero))
    novosDados = np.concatenate([deltaPosicao, np.matrix([[dados.data[3]]])], 0)

    lock.acquire()
    dadosGPS = np.concatenate([dadosGPS, novosDados], 1)
    lock.release()


def verificaDados():
    global dadosAcelerometro, dadosVelocidade, dadosGPS, lock
    global t, dt, dtIntervalo
    
    lock.acquire()
    if(dadosAcelerometro.shape[1] < 3):
        lock.release()
        return

    if(t == None):
        t = dadosAcelerometro[-1,0]
        for dado in [dadosVelocidade, dadosGPS]:
            try:
                if(t > dado[-1,0]): t = dado[-1,0]
            except:
                continue

    tminimo = t - dtIntervalo
    tmaximo = t + dtIntervalo
    dadosMesmoIntervalo = [None]*3

    if(dadosAcelerometro.shape[1] > 0):
        if(dadosAcelerometro[-1, 0] < tminimo):
            dadosAcelerometro = dadosAcelerometro[:, 1:]
        if(tminimo < dadosAcelerometro[-1, 0] and dadosAcelerometro[-1,0] <= tmaximo):
            dadosMesmoIntervalo[0] = dadosAcelerometro[:,0]
            dadosAcelerometro = dadosAcelerometro[:, 1:]
        else:
            dadosMesmoIntervalo[0] = dadosAcelerometro[:,0]
    
    if(dadosVelocidade.shape[1] > 0):
        if(dadosVelocidade[-1, 0] < tminimo):
            dadosVelocidade = dadosVelocidade[:, 1:]
        if(tminimo < dadosVelocidade[-1, 0] and dadosVelocidade[-1,0] <= tmaximo):
            dadosMesmoIntervalo[1] = dadosVelocidade[:,0]
            dadosVelocidade = dadosVelocidade[:, 1:]
    
    if(dadosGPS.shape[1] > 0):
        if(dadosGPS[-1, 0] < tminimo):
            dadosGPS = dadosGPS[:, 1:]
        if(tminimo < dadosGPS[-1, 0] and dadosGPS[-1,0] <= tmaximo):
            dadosMesmoIntervalo[2] = dadosGPS[:,0]
            dadosGPS = dadosGPS[:, 1:]

    lock.release()
    
    filtroKalman(dadosMesmoIntervalo)
    t += dt


def filtroKalman(dadosMesmoIntervalo):
    global t, dt, A, B, H, P, R, Rv, Rgps
    global Xcorrigido, Pcorrigido, dadosKalman
    global aceleracaoDadosKalman, velocidadeDadosKalman, posicaoDadosKalman

    dadoAcelerometro = None
    dadoVelocidade = None
    dadoGPS = None

    if(np.size(dadosMesmoIntervalo[0]) > 1):
        dadoAcelerometro = dadosMesmoIntervalo[0]

        aceleracaoDadosKalman[0].append(dadoAcelerometro[1,0])
        aceleracaoDadosKalman[1].append(dadoAcelerometro[-1,0])
    else:
        aceleracaoDadosKalman[0].append(None)
        aceleracaoDadosKalman[1].append(None)

    if(np.size(dadosMesmoIntervalo[1]) > 1):
        dadoVelocidade = dadosMesmoIntervalo[1]

        velocidadeDadosKalman[0].append(dadoVelocidade[0,0])
        velocidadeDadosKalman[1].append(dadoVelocidade[-1,0])
    else:
        velocidadeDadosKalman[0].append(None)
        velocidadeDadosKalman[1].append(None)

    if(np.size(dadosMesmoIntervalo[2]) > 1):
        dadoGPS = dadosMesmoIntervalo[2]

        posicaoDadosKalman[0].append(dadoGPS[1,0])
        posicaoDadosKalman[1].append(dadoGPS[-1,0])
    else:
        posicaoDadosKalman[0].append(None)
        posicaoDadosKalman[1].append(None)

    Xprevisto = None
    if(np.size(dadoAcelerometro) > 1):
        Xprevisto = A*Xcorrigido[:,-1] + B*dadoAcelerometro[1,0]
    else:
        Xprevisto = A*Xcorrigido[:,-1]

    Pprevisto = A*Pcorrigido*A.transpose() + P

    KG = None
    if(np.size(dadoVelocidade) > 1 and np.size(dadoGPS) > 1):
        KG = (Pprevisto*H.transpose())*np.linalg.inv(H*Pprevisto*H.transpose() + R)
    elif(np.size(dadoVelocidade) > 1):
        KG = (Pprevisto*H.transpose())*np.linalg.inv(H*Pprevisto*H.transpose() + Rv)
    elif(np.size(dadoGPS) > 1):
        KG = (Pprevisto*H.transpose())*np.linalg.inv(H*Pprevisto*H.transpose() + Rgps)
    else:
        KG = 0
    
    Y = None
    if(np.size(dadoVelocidade) > 1 and np.size(dadoGPS) > 1):
        Y = np.matrix([[dadoGPS[1,0]],[dadoVelocidade[0,0]]])
    elif(np.size(dadoVelocidade) > 1):
        Y = np.matrix([[Xprevisto[0,0]],[dadoVelocidade[0,0]]])
    elif(np.size(dadoGPS) > 1):
        Y = np.matrix([[dadoGPS[1,0]],[Xprevisto[1,0]]])
    else:
        Y = Xprevisto
    
    novoXcorrigido = Xprevisto + KG*(Y - H*Xprevisto)
    Xcorrigido = np.concatenate([Xcorrigido, novoXcorrigido], 1)
    Pcorrigido = Pprevisto - KG*H*Pprevisto

    dadosKalman[0].append(novoXcorrigido[0,0])
    dadosKalman[1].append(novoXcorrigido[1,0])
    dadosKalman[2].append(t)


if __name__ == "__main__":
    os.system('clear')
    rospy.init_node('kalman', anonymous=True)
    rospy.on_shutdown(desligandoNode)

    rospy.Subscriber('velocidadeMedida', Float32MultiArray, velocidadeMedidaCB)
    rospy.Subscriber('posicaoMedida', Float32MultiArray, gpsMedidoCB)
    rospy.Subscriber('acelerometroMedido', Float32MultiArray, acelerometroMedidoCB)

    rate = rospy.Rate(1/dt)
    while not rospy.is_shutdown():
        verificaDados()
        rate.sleep()