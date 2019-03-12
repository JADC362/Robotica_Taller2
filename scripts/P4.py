#!/usr/bin/env python

#Importacion de librerias
import sys, os, time
import rospy
import numpy as np

import threading
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import matplotlib.gridspec as gridspec

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

#Variable booleana que mantiene el hilo que grafica activo mientras no se cierre ninguna grafica 
hiloCerrado = False
hilo = 0

#Variable que representa el umbral de error aceptado en rho
errorRho = 0.05;
#Variable que representa si ya se sobrepaso el umbral de error
umbralSuperado = False

#Vector que contiene los parametros de la rueda 1 del robot: alpha, beta, r, l
paraRueda1 = [np.pi/2,np.pi,97.65/1000,44.5/100]
paraRueda2 = [-np.pi/2,0,97.65/1000,44.5/100]

#Matriz J1 que describe las restricciones de deslizamiento y rodamiento
J1 = np.array([[np.sin(paraRueda2[0]+paraRueda1[1]), -np.cos(paraRueda2[0]+paraRueda1[1]), -(paraRueda1[3])*np.cos(paraRueda1[1])],
	[np.sin(paraRueda1[0]+paraRueda2[1]), -np.cos(paraRueda1[0]+paraRueda2[1]), -(paraRueda1[3])*np.cos(paraRueda2[1])],
	[np.cos(paraRueda2[0]+paraRueda2[1]), np.sin(paraRueda2[0]+paraRueda2[1]), (paraRueda1[3])*np.sin(paraRueda2[1])]])

#Variables que representan la velocidad de cada motor del robot
velocidadM1 = 0
velocidadM2 = 0

#Variables que indican si se actualizo la informacion
callTime = False
callPos = False

#Posicion final del robot. Posee un valor por defecto
posicionFinalCar = [40.0,40.0,-np.pi/2]
#Posicion actual del robot. Posee un valor por defecto
posicionActualCar = [0.0,0.0,-np.pi]

#Variable que representa el tiempo de ejecucion
simulationTime = 0 #Tiempo actual
simulationTimeAnterior = 0 #Tiempo anterior
pasoDeSimulacion = simulationTime-simulationTimeAnterior #Paso de simulacion utilizado por vrep

#Vectores de posicion actual del robot simulado
posicionXActualSimulada = [0]
posicionYActualSimulada = [0]

#Vevtores de posicion actual del robot teorico
posicionXActualExacta = [0]
posicionYActualExacta = [0]

#Vectores utilizados para el calculo del error absoluto
Error = []
tiempo = []


#Funcion callback llamada cuando el topico pionnerPosition es actualizado
#En esta funcion se actualiza la posicion actual del robot simulado y se calcula la posicion teorica del robot
#ademas en caso de que sea la primera vez que se ejecuta la funcion se inicializa la posicion inicial del robot 
def callbackPioneerPosition(msg):

	global posicionActualCar, callPos, posicionXActualSimulada, posicionYActualSimulada, posicionXActualExacta, posicionYActualExacta
	posicionActualCar = [msg.linear.x,msg.linear.y,msg.angular.z]
	callPos = True

	posicionXActualSimulada.append(posicionActualCar[0])
	posicionYActualSimulada.append(posicionActualCar[1])

	calcularRecorridoExacto()

#Funcion callback llamada cuando el topico simulationTime es actualizado
#La funcion se encarga de actualizar el tiempo actual, el tiempo anterior, el paso de simulacion y el tiempo que lleva la velocidad actual en el robot
def callbackSimulationTime(msg):
	global simulationTime,pasoDeSimulacion, callTime

	if simulationTime == 0:
		simulationTime = msg.data

	callTime = True
	simulationTimeAnterior=simulationTime
	simulationTime = msg.data
	pasoDeSimulacion = simulationTime-simulationTimeAnterior
	

#Obtiene la posicion del robot en coordenadas polares rho, alpha, beta a partir de la entrada en posicion cartesianas
def obtenerPosicionPol():
	global posicionFinalCar, umbralSuperado, errorRho, posicionActualCar
	rho = np.sqrt((posicionFinalCar[0]-posicionActualCar[0])**2+(posicionFinalCar[1]-posicionActualCar[1])**2)

	if rho <= errorRho:
		umbralSuperado = True

	if not umbralSuperado:
		alpha = -posicionActualCar[2]+np.arctan2((posicionFinalCar[1]-posicionActualCar[1]),(posicionFinalCar[0]-posicionActualCar[0]))
	else:
		alpha = 0;
		rho = 0;
	beta = -alpha-posicionActualCar[2]

	return np.asarray([rho,alpha,beta])

#Funcion encargada de determinar las velocidades de los motores en cinametica inversa a partir de una ley de control
def calcularCinematicaRobot():
	global posicionActualCar, velocidadM1, velocidadM2, paraRueda1, paraRueda2

	k = [0.02,0.4,0.01]
	
	#Obtencion del error de posicion en coordenadas polares
	posPol = np.asarray([0,0,-posicionFinalCar[2]])-obtenerPosicionPol()
	#rospy.loginfo(posPol) #Se imprime en consola el error

	#Ley de control aplicada para encontrar v y w
	vecVelLinearAngular = np.asarray([k[0]*posPol[0],k[1]*posPol[1]+k[2]*posPol[2]])
	
	#A partir de v y w se determina el vector [x',y',theta']
	veloCar = [(vecVelLinearAngular[0])*(np.cos(posicionActualCar[2])),(vecVelLinearAngular[0])*(np.sin(posicionActualCar[2])),vecVelLinearAngular[1]]

	#Matriz de transformacion del marco global al local
	matrixR = [[np.cos(posicionActualCar[2]),np.sin(posicionActualCar[2]),0],[-np.sin(posicionActualCar[2]),np.cos(posicionActualCar[2]),0],[0,0,1]]
	#Obtencion de las velocidades aplicadas a cada motor. Cinematica aplicada
	velocidadM1 = np.dot([np.sin(paraRueda1[0]+paraRueda1[1]),-np.cos(paraRueda1[0]+paraRueda1[1]),-(paraRueda1[3])*(np.cos(paraRueda1[1]))],np.dot(matrixR,veloCar))/paraRueda1[2]
	velocidadM2 = np.dot([np.sin(paraRueda2[0]+paraRueda2[1]),-np.cos(paraRueda2[0]+paraRueda2[1]),-(paraRueda2[3])*(np.cos(paraRueda2[1]))],np.dot(matrixR,veloCar))/paraRueda2[2]

#Funcion que grafica la posicion actual del pioneer y su correspondiente error (simulado vs teorico)
def graficar():
	global hiloCerrado,hilo
	plt.ion()
	wfig = 10
	hfig = 8
	proporcionTitulo = 0.9
	proporcionLabels = 0.7
	fig = plt.figure(figsize=(wfig,hfig))
	gs = gridspec.GridSpec(1, 2)
	gs.update(hspace=10);
	while not hiloCerrado:

		try: 
			plt.clf()
			ax0 = fig.add_subplot(gs[:, 0])
			ax0.plot(posicionXActualSimulada,posicionYActualSimulada,'m--', linewidth = 2)
			ax0.plot(posicionXActualExacta,posicionYActualExacta,'r:', linewidth = 2)
			ax0.scatter(posicionXActualSimulada[-1],posicionYActualSimulada[-1], s=20**2, c= 250, alpha = 0.6 )
			ax0.set_xlabel("Posicion en el eje X del robot con respecto al marco global [m]",fontsize = wfig*proporcionLabels)
			ax0.set_ylabel("Posicion en el eje Y del robot con respecto al marco global [m]", fontsize = wfig*proporcionLabels)
			plt.title("Posicion simulada en tiempo real del robot", fontsize = wfig*proporcionTitulo)
			ax0.grid(True)
			ax0.scatter(posicionXActualExacta[-1],posicionYActualExacta[-1], s=10**2, c= 500, alpha = 0.4 )
			fig.savefig(os.getcwd()+"/src/taller2_5/results/GraficasPunto3.png")
			ax1 = fig.add_subplot(gs[:, 1])
			ax1.plot(tiempo,Error,'r--',linewidth=2)
			ax1.set_xlabel("Tiempo[sg]",fontsize = wfig*proporcionLabels)
			ax1.set_ylabel("Error absoluto[m]", fontsize = wfig*proporcionLabels)
			plt.title("Error absoluto entre la posicion del robot simulada y la teorica", fontsize = wfig*proporcionTitulo)
			ax1.grid(True)
			fig.canvas.draw()
			fig.canvas.flush_events()
			plt.pause(0.01)
			
		except Exception as e:
			plt.close('all')
			hiloCerrado = True
			break

	return False
	
#Funcion encargada de calcular la posicion teorica del robot asi como el error actual 
def calcularRecorridoExacto():

	global posicionXActualExacta,posicionYActualExacta, Error, paraRueda1, velocidadM1, velocidadM2, J1, posicionActualCar, pasoDeSimulacion, tiempo
	try:
		R = np.array([[np.cos(posicionActualCar[2]), np.sin(posicionActualCar[2]), 0],[-np.sin(posicionActualCar[2]), np.cos(posicionActualCar[2]),0],[0 ,0 ,1]])
		phi = [[(paraRueda1[2])*velocidadM1],[(paraRueda1[2])*velocidadM2],[0]] #Vector phi que lleva las velocidades de los motores
		J1inv = np.linalg.inv(J1)
		Rinv = np.linalg.inv(R)
		velocidadesGlobales = Rinv.dot(J1inv.dot(np.array(phi))) #Velocidades lineal y angular actual del robot

		#Calculo de la posicion actual
		xExacta = (velocidadesGlobales[0]*pasoDeSimulacion+posicionXActualExacta[-1])
		yExacta = (velocidadesGlobales[1]*pasoDeSimulacion+posicionYActualExacta[-1])

		posicionXActualExacta.append(xExacta)
		posicionYActualExacta.append(yExacta)

		#Calculo del error 
		xSimulada = posicionXActualSimulada[-1]
		ySimulada = posicionYActualSimulada[-1]
		ErrorTotal = np.sqrt((xSimulada-xExacta)**2+(ySimulada-yExacta)**2)
		Error.append(ErrorTotal)
		tiempo.append(simulationTime)

	except Exception as e:
		pass

#Funcion principal de programa
def main():
	global posicionFinalCar, velocidadM1, velocidadM2, callTime, callPos, hilo

	#Obtencion del vector de posicion final del robot. Si no se envia se toma por defecto [40,40,pi/2]
	if len(sys.argv) > 1:
		posicionFinalCar = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])]

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
		hilo=threading.Thread(target= graficar, name='Graficar')
		hilo.start()
		time.sleep(0.1)
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
