function sysCall_init()
    -- Valores iniciais de simulação
    relacaoEncoder = 0.0574
    velocidadeDesejada = 0
    vEntrada = {0}
    vSaida = {0, 0}

    motorEsquerdoHanlder = sim.getObjectHandle('motorDianteiroEsquerdo')
    motorDireitoHandler = sim.getObjectHandle('motorDianteiroDireito')

    sim.setJointTargetVelocity(motorEsquerdoHanlder, velocidadeDesejada)
    sim.setJointTargetVelocity(motorDireitoHandler, velocidadeDesejada)
    
    sim.setJointMaxForce(motorEsquerdoHanlder, 0.3)
    sim.setJointMaxForce(motorDireitoHandler, 0.3)

    if simROS then
        motorSub = simROS.subscribe('/motor','std_msgs/Float32','velocidadeRadianoCb')
        result = sim.launchExecutable('motorSub', motorSub, 0)
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
end


function sysCall_actuation()
    vEntrada[1] = velocidadeDesejada
    vSaida[2] = vSaida[1]

    if velocidadeDesejada > vSaida[1] then
        vSaida[1] = 0.1813*velocidadeDesejada + 0.8187*vSaida[2]
    else
        vSaida[1] = 0.07688*velocidadeDesejada + 0.9231*vSaida[2]
    end

    sim.setJointTargetVelocity(motorEsquerdoHanlder, vSaida[1])
    sim.setJointTargetVelocity(motorDireitoHandler, vSaida[1])

end


function sysCall_cleanup()
    simROS.shutdownSubscriber(motorSub)
end


function velocidadeRadianoCb(msg)
    velocidadeDesejada = msg.data/relacaoEncoder
end
