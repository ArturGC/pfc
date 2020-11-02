function sysCall_init()
    -- Valores iniciais de simulação
    tempoZero = simROS.getTime()
    
    rodaEsquerdaHandle = sim.getObjectHandle('eixoTraseiroEsquerdo')
    rodaDireitaHandle = sim.getObjectHandle('eixoTraseiroDireito')

    sim.setJointPosition(rodaEsquerdaHandle, 0)
    rodaEsquerdaPosAnterior = 0
    rodaEsquerdaPosAtual = 0
    
    sim.setJointPosition(rodaDireitaHandle, 0)
    rodaDireitaPosAnterior = 0
    rodaDireitaPosAtual = 0

    -- Velocidade pelo sistema 
    veiculoHandler = sim.getObjectHandle('veiculo')

    if simROS then
        encoderPub = simROS.advertise('/encoder','std_msgs/Float32MultiArray')
        result = sim.launchExecutable('encoderPub', encoderPub, 0)
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
end


function sysCall_sensing()
    tempoAtual = simROS.getTime() - tempoZero
    
    rodaEsquerdaPosAtual = sim.getJointPosition(rodaEsquerdaHandle)
    rodaDireitaPosAtual = sim.getJointPosition(rodaDireitaHandle)

    dxRodaEsquerda = calcularDeslocamentoRad(rodaEsquerdaPosAtual, rodaEsquerdaPosAnterior)
    dxRodaDireita = calcularDeslocamentoRad(rodaDireitaPosAtual, rodaDireitaPosAnterior)

    rodaEsquerdaPosAnterior = rodaEsquerdaPosAtual
    rodaDireitaPosAnterior = rodaDireitaPosAtual

    dadosEncoder = {}
    dadosEncoder['data'] = {(dxRodaDireita + dxRodaEsquerda)/2, tempoAtual}
    simROS.publish(encoderPub, dadosEncoder)
end


function sysCall_cleanup()
    simROS.shutdownPublisher(encoderPub)
end


function calcularDeslocamentoRad(angle,alpha)
    sinAngle0 = math.sin(angle)
    sinAngle1 = math.sin(alpha)
    cosAngle0 = math.cos(angle)
    cosAngle1 = math.cos(alpha)
    sin_da = sinAngle0*cosAngle1-cosAngle0*sinAngle1
    cos_da = cosAngle0*cosAngle1+sinAngle0*sinAngle1
    return(math.atan2(sin_da,cos_da))
end

