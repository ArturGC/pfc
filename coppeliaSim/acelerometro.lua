function sysCall_init()
    -- Valores iniciais de simulação
    tempoZero = simROS.getTime()

    massObject = sim.getObjectHandle('Accelerometer_mass')
    acelerometroHanlder = sim.getObjectHandle('Accelerometer_forceSensor')
    result,mass = sim.getObjectFloatParameter(massObject,sim.shapefloatparam_mass)

    if simROS then
        acelerometroPub = simROS.advertise('/acelerometro', 'std_msgs/Float32MultiArray')
        result = sim.launchExecutable('acelerometroPub', acelerometroPub, 0)
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
end


function sysCall_sensing()
    tempoAtual = simROS.getTime() - tempoZero

    result,force = sim.readForceSensor(acelerometroHanlder)
    dadosAcelerometro = {}
    
    if result == 0 then
        dadosAcelerometro['data'] = {0, 0, 0, tempoAtual}
        simROS.publish(acelerometroPub, dadosAcelerometro)
    else
        dadosAcelerometro['data'] = {force[1]/mass, force[2]/mass, force[3]/mass, tempoAtual}
        simROS.publish(acelerometroPub, dadosAcelerometro)
    end
end


function sysCall_cleanup()
    simROS.shutdownPublisher(acelerometroPub)
end    


