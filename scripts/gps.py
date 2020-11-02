#! /usr/bin/env python3
import rospy
import csv
import random
from std_msgs.msg import Float32MultiArray


# Variáveis Globais
i = 0
posicaoMedida = [[],[],[],[]]
posicaoReal = [[],[],[],[]]


def gpsCB(dados, gpsPub):
    global i, posicaoMedida, posicaoReal

    posicaoReal[0].append(dados.data[0])
    posicaoReal[1].append(dados.data[1])
    posicaoReal[2].append(dados.data[2])
    posicaoReal[3].append(dados.data[3])

    mu = 0
    sigma = 0.4
    amplitudeMax = 2.5
    posicaoMedida[0].append(dados.data[0] + amplitudeMax*random.gauss(mu, sigma))
    posicaoMedida[1].append(dados.data[1] + amplitudeMax*random.gauss(mu, sigma))
    posicaoMedida[2].append(dados.data[2] + amplitudeMax*random.gauss(mu, sigma))
    posicaoMedida[3].append(dados.data[3])

    i += 1
    j = random.random()
    if(i > 10 and j > 0.90 or i > 30):
        dadosPub = Float32MultiArray()
        dadosPub.data = [posicaoMedida[0][-1], posicaoMedida[1][-1], posicaoMedida[2][-1], posicaoMedida[3][-1]] 
        gpsPub.publish(dadosPub)
        i = 0
    

def desligandoNode():
    global posicaoMedida, posicaoReal

    diretorio = rospy.get_param('/dadosExecucao')[0]
    fileName = '{}/posicaoMedida.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in posicaoMedida:
            writer.writerow(lista)

    diretorio = rospy.get_param('/dadosExecucao')[0]
    fileName = '{}/posicaoReal.csv'.format(diretorio)
    with open(fileName, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for lista in posicaoReal:
            writer.writerow(lista)


if __name__ == "__main__":
    rospy.init_node('gpsSub', anonymous=True)
    rospy.on_shutdown(desligandoNode)

    gpsPub = rospy.Publisher('posicaoMedida', Float32MultiArray, queue_size=10)
    rospy.Subscriber('gps', Float32MultiArray, gpsCB, gpsPub)
    
    rospy.spin()
