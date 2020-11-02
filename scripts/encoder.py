#! /usr/bin/env python3
import rospy
import csv
import random
from std_msgs.msg import Float32MultiArray


# Variáveis Globais
i = 0
encoderReal = [[0],[0]]
velocidadeReal = [[],[]]
encoderMedido = [[0],[0]]
velocidadeMedida = [[],[]]


def encoderCB(dados, velocidadePub):
    global i, encoderReal, encoderMedido, velocidadeReal, velocidadeMedida
    relacaoEncoder = 0.0574
    
    encoderReal[0].append(dados.data[0])
    encoderReal[1].append(dados.data[1])

    velocidadeReal[0].append((relacaoEncoder * encoderReal[0][-1])/(encoderReal[1][-1] -encoderReal[1][-2]))
    velocidadeReal[1].append(dados.data[1])

    sigma = 0.5
    amplitudeMax = 0.35
    mu = 0
    encoderMedido[0].append(dados.data[0] + amplitudeMax*random.gauss(mu, sigma))
    encoderMedido[1].append(dados.data[1])

    velocidadeMedida[0].append((relacaoEncoder * encoderMedido[0][-1])/(encoderMedido[1][-1] - encoderMedido[1][-2]))
    velocidadeMedida[1].append(dados.data[1])

    i += 1
    j = random.random()
    if(i > 1 and j > 0.5 or i > 2):
        dadosPub = Float32MultiArray()
        dadosPub.data = [velocidadeMedida[0][-1], velocidadeMedida[1][-1]]
        velocidadePub.publish(dadosPub)
        i = 0
    

def desligandoNode():
    global velocidadeReal, velocidadeMedida
    diretorio = rospy.get_param('/dadosExecucao')[0]
    
    fileName = '{}/velocidadeMedida.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in velocidadeMedida:
            writer.writerow(lista)

    fileName = '{}/velocidadeReal.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in velocidadeReal:
            writer.writerow(lista)


if __name__ == "__main__":    
    rospy.init_node('encoderSub', anonymous=True)
    rospy.on_shutdown(desligandoNode)

    velocidadePub = rospy.Publisher('velocidadeMedida', Float32MultiArray, queue_size=10)
    rospy.Subscriber('encoder', Float32MultiArray, encoderCB, velocidadePub)

    rospy.spin()
