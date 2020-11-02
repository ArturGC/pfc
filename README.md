## Funcionamento do código

- A pasta "pfc" é um package do ROS e deve ser coloca dentro da pasta "src";
- Para que seja possível executar toda a simulação é necessário ter instalado as bibliotecas de comunicação entre o CoppeliaSim e o ROS: https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm;
- Os códigos .lua e arquivo do cenário referentes ao CoppeliaSim estão dentro da pasta "coppeliaSim";
- Os códigos ROS estão dentro da pasta "script";
- A pasta "dados" é onde todas os dados coletados durante a simulação são salvos;
- O arquivo "sistema_launch.py" irá iniciar a simulação do CoppeliaSim e os nós ROS;
- Para executar a simulação completa junto com o Filtro de Kalman faça os seguintes passos:
  1. Abra uma janela de terminal e inicialize o "roscore";
  2. Abra outra janela de teminal e inicializa o CoppeliaSim do mesmo modo que é apresentando no link tutorial acima;
  3. Abre o cenário "coppeliaSim/veiculo.ttt" no CoppeliaSim;
  4. Em uma nova janela do terminal execute o comando: rosrun pfc sistema_launch.py
