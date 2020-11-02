function sysCall_init()
    -- Valores iniciais de simulação
    tempoZero = simROS.getTime()
    
    veiculoHandler = sim.getObjectHandle('veiculo')

    if simROS then
        gpsPub = simROS.advertise('/gps', 'std_msgs/Float32MultiArray')
        result = sim.launchExecutable('gpsPub', gpsPub, 0)
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
end


function sysCall_sensing()
    tempoAtual = simROS.getTime() - tempoZero

    posicaoAtual = sim.getObjectPosition(veiculoHandler, -1)
    posicaoAtual[4] = tempoAtual
    
    dadosPosicao = {}
    dadosPosicao['data'] = posicaoAtual
    
    simROS.publish(gpsPub, dadosPosicao)
end


function sysCall_cleanup()
    simROS.shutdownPublisher(gpsPub)
end
