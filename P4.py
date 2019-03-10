#!/usr/bin/env python

#Importacion de librerias
import sys 
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

linear = Vector3()
angular = Vector3()

#Variable que representa el umbral de error aceptado en rho
errorRho = 0.05;
#Variable que representa si ya se sobrepaso el umbral de error
umbralSuperado = False

#Vector que contiene los parametros de la rueda 1 del robot: alpha, beta, r, l
paraRueda1 = [np.pi/2,-np.pi,97.65/1000,44.5/100]
paraRueda2 = [-np.pi/2,0,97.65/1000,44.5/100]

#Variables que representan la velocidad de cada motor del robot
velocidadM1 = 0
velocidadM2 = 0

#Variables que indican si se actualizo la informacion
callTime = False
callPos = False

#Posicion inicial del robot. Posee un valor por defecto
posicionInicialCar = [0.0,-np.pi]
#Posicion final del robot. Posee un valor por defecto
posicionFinalCar = [-40.0,-40.0,-np.pi/2]
#Posicion actual del robot. Posee un valor por defecto
posicionActualCar = [0.0,0.0,0.0]

#Variable que representa el tiempo de simulacion de VREP
simulationTime = 0;

#Funcion callback llamada cuando se actualiza el topico pioneerPosition
def callbackPioneerPosition(msg):
	global posicionActualCar, callPos
	posicionActualCar = [msg.linear.x,msg.linear.y,msg.angular.z]
	callPos = True
	

#Funcion callback llamada cuando se actualiza el topico simulationTime
def callbackSimulationTime(msg):
	global simulationTime, callTime
	simulationTime = msg.data;
	callTime = True

#Obtiene la posicion del robot en coordenadas polares rho, alpha, beta
def obtenerPosicionPol(posCar):
	global posicionFinalCar, umbralSuperado, errorRho
	rho = np.sqrt((posicionFinalCar[0]-posCar[0])**2+(posicionFinalCar[1]-posCar[1])**2)

	if rho <= errorRho:
		umbralSuperado = True

	if not umbralSuperado:
		alpha = -posCar[2]+np.arctan2((posicionFinalCar[1]-posCar[1]),(posicionFinalCar[0]-posCar[0]))
	else:
		alpha = 0;
		rho = 0;
	beta = -alpha-posCar[2]

	return np.asarray([rho,alpha,beta])

#Funcion encargada de determinar las velocidades de los motores en cinametica inversa a partir de una ley de control
def calcularCinematicaRobot():
	global posicionActualCar, velocidadM1, velocidadM2, paraRueda1, paraRueda2

	k = [0.02,0.4,0.01]
	
	#Obtencion de la posicion en polares	
	posPol = np.asarray([0,0,-posicionFinalCar[2]])-obtenerPosicionPol(posicionActualCar)

	#Obtencion del vector de velocidades
	#veloPolMatrix = np.tnp.pi,ranspose([[np.cos(posPol[1]),-np.sin(posPol[1])/posPol[0],np.sin(posPol[1])/posPol[0]],[0,1,0]])
	#veloCarMatrix = np.transpose([[np.cos(posicionActualCar[2]),np.sin(posicionActualCar[2]),0],[0,0,1]])
	#if -np.pi/2 <= posPol[1] <= np.pi/2:
	#	veloPolMatrix = -veloPolMatrix
	rospy.loginfo(posPol)
	vecVelLinearAngular = np.asarray([k[0]*posPol[0],k[1]*posPol[1]+k[2]*posPol[2]])
	

	veloCar = [(vecVelLinearAngular[0])*(np.cos(posicionActualCar[2])),(vecVelLinearAngular[0])*(np.sin(posicionActualCar[2])),vecVelLinearAngular[1]]
	#veloPol = veloPolMatrix.dot(vecVelLinearAngular)
	#veloCar = veloCarMatrix.dot(vecVelLinearAngular)

	matrixR = [[np.cos(posicionActualCar[2]),np.sin(posicionActualCar[2]),0],[-np.sin(posicionActualCar[2]),np.cos(posicionActualCar[2]),0],[0,0,1]]
	velocidadM1 = np.dot([np.sin(paraRueda1[0]+paraRueda1[1]),-np.cos(paraRueda1[0]+paraRueda1[1]),-(paraRueda1[3])*(np.cos(paraRueda1[1]))],np.dot(matrixR,veloCar))/paraRueda1[2]
	velocidadM2 = np.dot([np.sin(paraRueda2[0]+paraRueda2[1]),-np.cos(paraRueda2[0]+paraRueda2[1]),-(paraRueda2[3])*(np.cos(paraRueda2[1]))],np.dot(matrixR,veloCar))/paraRueda2[2]
#Funcion principal de programa
def main():
	global posicionFinalCar, velocidadM1, velocidadM2, callTime, callPos

	#Obtencion del vector de posicion final del robot. Si no se envia se toma por defecto [40,40,pi/2]
	if len(sys.argv) > 1:
		posicionFinalCar = sys.argv[1]

	#Se inicializa un nodo con el nombre P4_CinematicaRobot
	rospy.init_node('P4_CinematicaRobot',anonymous=False)

	#Publicacion al topico de velocidades para los motores del robot pioneer
	pubMotorsVel = rospy.Publisher('motorsVel',Float32MultiArray,queue_size=10)

	#Suscripcion al topico pioneerPosition
	rospy.Subscriber("pioneerPosition",Twist,callbackPioneerPosition)
	rospy.Subscriber("simulationTime",Float32,callbackSimulationTime)

	#Tiempo durante el cual duerme el nodo
	rate = rospy.Rate(20)

	

	try:
		
		#Mientras el nodo este en ejecucion
		while not rospy.is_shutdown():

			if callTime and callPos:
				calcularCinematicaRobot()
				callTime = False
				callPos = False

			#Publica la velocidad del robot en el topico motorsVel
			mensaje = Float32MultiArray(data=[velocidadM1,velocidadM2])
			pubMotorsVel.publish(mensaje)

			#Se envia a dormir al nodo
			rate.sleep()

	except Exception as e:
		raise e


if __name__ == '__main__':
	main()
else:
	rospy.loginfo("Fallo al cargar codigo")