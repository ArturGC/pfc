#! /usr/bin/env python3
import os
import roslaunch
import rospy

rospy.init_node('sistema', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
path = '{}/launch/sistema.launch'.format(os.getcwd())
launch = roslaunch.parent.ROSLaunchParent(uuid, [path])

sinal = [0]*1 + [1]*3 + [1.8]*4 + [0.5]*2 + [0]*1
sinal += [0]*1 + [-1]*3 + [-2]*4 + [-0.5]*2 + [0]*1
# sinal += [0]*1 + [1]*3 + [2]*4 + [0.5]*2 + [0]*1
# sinal += [0]*1 + [-1]*3 + [-2]*4 + [-0.5]*2 + [0]*1
# sinal += [0]*1 + [1]*3 + [2]*4 + [0.5]*2 + [0]*1
# sinal += [0]*1 + [-1]*3 + [-2]*4 + [-0.5]*2 + [0]*1


rospy.set_param('/dadosExecucao',['{}/dados'.format(os.getcwd())])
rospy.set_param('/sinalMotor', sinal)

os.system('clear')

print("\n### Iniciando CoppeliaSim ###\n")
os.system('rostopic pub /startSimulation std_msgs/Bool true --once')
rospy.sleep(2)

print("\n### Iniciando ROS ###\n")
launch.start()

rospy.sleep(2*(len(sinal)))

print("\n### Finalizando ROS ###\n")
launch.shutdown()

print("\n### Finalizando CoppeliaSim ###\n")
os.system('rostopic pub /stopSimulation std_msgs/Bool true --once')
