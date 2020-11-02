#! /usr/bin/env python3
import rospy
import csv
import random
from std_msgs.msg import Float32MultiArray


# Variáveis Globais
acelerometroReal = [[],[],[],[]]
acelerometroMedido = [[],[],[],[]]


def desligandoNode():
    global acelerometroMedido, acelerometroReal
    diretorio = rospy.get_param('/dadosExecucao')[0]
    
    fileName = '{}/acelerometroMedido.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in acelerometroMedido:
            writer.writerow(lista)
    
    fileName = '{}/acelerometroReal.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in acelerometroReal:
            writer.writerow(lista)


def acelerometroCB(dados): 
    global acelerometroReal, acelerometroMedido

    mu = 0
    sigma = 0.5
    amplitudeMax = 0.35
    
    acelerometroReal[0].append(dados.data[0])
    acelerometroReal[1].append(dados.data[1])
    acelerometroReal[2].append(dados.data[2])
    acelerometroReal[3].append(dados.data[3])

    acelerometroMedido[0].append(dados.data[0] + amplitudeMax*random.gauss(mu, sigma))
    acelerometroMedido[1].append(dados.data[1] + amplitudeMax*random.gauss(mu, sigma))
    acelerometroMedido[2].append(dados.data[2] + amplitudeMax*random.gauss(mu, sigma))
    acelerometroMedido[3].append(dados.data[3])

    dadosPub = Float32MultiArray()
    dadosPub.data = [acelerometroMedido[0][-1], acelerometroMedido[1][-1], acelerometroMedido[2][-1], acelerometroMedido[3][-1]]
    acelerometroPub.publish(dadosPub)
    

if __name__ == "__main__":
    rospy.init_node('acelerometroSub', anonymous=True)
    rospy.on_shutdown(desligandoNode)
    
    rospy.Subscriber('acelerometro', Float32MultiArray, acelerometroCB)
    acelerometroPub = rospy.Publisher('acelerometroMedido', Float32MultiArray, queue_size=10)
    
    rospy.spin()
