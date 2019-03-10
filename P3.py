#!/usr/bin/env python

#Importacion de librerias
import sys, os
import rospy
import numpy as np
import threading
import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32


linear = Vector3()
angular = Vector3()
hiloCerrado = False

#Especificaciones del chasis del robot
length = 0.445 
width = 0.393 

#Especificaciones de las llantas del robot
diameter = 0.19532
radio = diameter/2 
WheelsWidth = 0.0475

#Distancia del punto P puesto en el chasis a el punto de contacto de las llantas 
l = (width/2)+WheelsWidth/2

#Parametros alpha y beta de las llantas
alpha1 = -np.pi/2
alpha2 = np.pi/2
beta1 = np.pi
beta2 = 0 
tetha = 0

#Posicion inicial del robot
posicionInicialX=0.02736
posicionInicialY=0.44998
posicionInicialZ=0.13866

#Matriz J1 que describe las restricciones de deslizamiento y rodamiento

J1 = np.array([[np.sin(float(alpha1+beta1)), -np.cos(float(alpha1+beta1)), -l*np.cos(float(beta1))],
	[np.sin(float(alpha2+beta2)), -np.cos(float(alpha2+beta2)), -l*np.cos(float(beta2))],
	[np.cos(float(alpha2+beta2)), np.sin(float(alpha2+beta2)), l*np.sin(float(beta2))]])

#Vector de velocidades y tiempos entregados por el archivo de texto
velocidadesX = []
velocidadesY = []
tiempoVelocidades = []

#Variables que representan las velocidades actuales implementadas en el sistema
velocidadActualM1 = 0.0
velocidadActualM2 = 0.0

#Variable que representa el tiempo de ejecucion
simulationTimeB = 0 #Tiempo base
simulationTime = 0 #Tiempo actual
simulationTimeAnterior = 0
pasoDeSimulacion = simulationTime-simulationTimeAnterior
deltaTiempoVelocidades = simulationTime-simulationTimeB #Tiempo que lleva la velocidad actual ejecutandose

#Contador de velocidad actual
contadorVelAc = 0

#Vectores de posicion actual del robot simulado
posicionXActualSimulada = []
posicionYActualSimulada = []
posicionZActualSimulada = []

#Vevtores de posicion actual del robot teorico
posicionXActualExacta = [posicionInicialX]
posicionYActualExacta = [posicionInicialY]
posicionZActualExacta = [posicionInicialZ]

#Vectores utilizados para el calculo del error absoluto
Error = []
tiempo = []

#Hilo utilizado para graficar 
hilo = 0


#Funcion callback llamada cuando el topico pionnerPosition es actualizado
def callbackPioneerPosition(msg):

	global linear,angular,tetha,posicionInicialX,posicionInicialY,posicionInicialZ
	linear = msg.linear 
	angular = msg.angular
	x = linear.x 
	y = linear.y
	z = linear.z
	tetha = angular.z
	posicionXActualSimulada.append(x)
	posicionYActualSimulada.append(y)
	posicionZActualSimulada.append(z)

	if simulationTimeB == 0:

		posicionInicialX=posicionXActualSimulada[0]
		posicionInicialY=posicionYActualSimulada[0]
		posicionInicialZ=posicionZActualSimulada[0]

		posicionXActualExacta[0] = posicionInicialX
		posicionYActualExacta[0] = posicionInicialY
		posicionZActualExacta[0] = posicionInicialZ

	if len(posicionXActualSimulada) > 2:
		calcularRecorridoExacto()


#Funcion callback llamada cuando el topico simulationTime es actualizado
def callbackSimulationTime(msg):
	global simulationTimeB, simulationTime, deltaTiempoVelocidades,pasoDeSimulacion

	if simulationTimeB == 0:
		simulationTimeB = msg.data

	deltaTiempoVelocidades = simulationTime-simulationTimeB
	simulationTimeAnterior=simulationTime
	simulationTime = msg.data
	pasoDeSimulacion = simulationTime-simulationTimeAnterior

#Funcion que determina la velocidad actual que se va a implementar en el robot dependiendo del tiempo
def determinarVelocidadActual():
	global simulationTimeB, simulationTime, velocidadActualM1, velocidadActualM2, contadorVelAc, velocidadesX, velocidadesY, tiempoVelocidades

	if contadorVelAc < len(velocidadesX):
		velocidadActualM1 = velocidadesX[contadorVelAc]
		velocidadActualM2 = velocidadesY[contadorVelAc]
		if tiempoVelocidades[contadorVelAc]<=(deltaTiempoVelocidades):
			contadorVelAc += 1
			simulationTimeB = simulationTime
	else:
		velocidadActualM1 = 0
		velocidadActualM2 = 0
	

#Funcion que grafica la posicion actual del pioneer
def graficar():
	global hiloCerrado,hilo
	while not hiloCerrado:
		try:
			
			plt.figure(1)
			plt.clf()
			plt.plot(posicionXActualSimulada,posicionYActualSimulada,'m--', linewidth = 2)
			plt.scatter(posicionXActualSimulada[len(posicionXActualSimulada)-1],posicionYActualSimulada[len(posicionYActualSimulada)-1], s=20**2, c= 250, alpha = 0.6 )
			plt.xlabel("Posicion en el eje X del robot con respecto al marco global",fontsize = 13)
			plt.ylabel("Posicion en el eje Y del robot con respecto al marco global", fontsize = 13)
			plt.title("Posicion simulada en tiempo real del robot", fontsize = 18)
			plt.grid(True)
			plt.plot(posicionXActualExacta,posicionYActualExacta,'r:', linewidth = 2)
			plt.scatter(posicionXActualExacta[len(posicionXActualExacta)-1],posicionYActualExacta[len(posicionYActualExacta)-1], s=10**2, c= 500, alpha = 0.4 )
			plt.savefig(os.getcwd()+"/src/Taller2/resources/punto3.png")
		except Exception as e:
			plt.close('all')
			hiloCerrado = True

		try:
			plt.figure(2)
			plt.xlabel("Tiempo[sg]",fontsize = 13)
			plt.ylabel("Error absoluto[m]", fontsize = 13)
			plt.title("Error absoluto entre la posicion del robot simulada y la teorica", fontsize = 18)
			plt.grid(True)
			plt.plot(tiempo,Error,'r--',linewidth=2)
			plt.draw()
			plt.pause(0.1)
		except Exception as e:
			plt.close('all')
			hiloCerrado = True

	

	return False
	

def calcularRecorridoExacto():

	global posicionXActualExacta,posicionYActualExacta,posicionZActualExacta, Error,radio,velocidadActualM1,velocidadActualM2, J1
	try:
		Rinv = np.array([[np.cos(tetha), -np.sin(tetha), 0],
		[np.sin(tetha), np.cos(tetha),0],[0 ,0 ,1]])
		phi = [[radio*velocidadActualM1],[radio*velocidadActualM2],[0]]
		J1inv = np.linalg.inv(J1)
		velocidadesGlobales = Rinv.dot(J1inv.dot(np.array(phi)))

		xExacta = (velocidadesGlobales[0]*pasoDeSimulacion+posicionXActualExacta[len(posicionXActualExacta)-1])
		yExacta = (velocidadesGlobales[1]*pasoDeSimulacion+posicionYActualExacta[len(posicionYActualExacta)-1])
		zExacta = (velocidadesGlobales[2]*pasoDeSimulacion+posicionZActualExacta[len(posicionZActualExacta)-1])

		posicionXActualExacta.append(xExacta)
		posicionYActualExacta.append(yExacta)
		posicionZActualExacta.append(zExacta)

		xSimulada = posicionXActualSimulada[len(posicionXActualSimulada)-1]
		ySimulada = posicionYActualSimulada[len(posicionYActualSimulada)-1]

		ErrorTotal = np.sqrt((xSimulada-xExacta)**2+(ySimulada-yExacta)**2)
		Error.append(ErrorTotal)
		tiempo.append(simulationTime)

	except Exception as e:
		raise e
		pass



#Funcion principal del programa
def main():

	global velocidadesX, velocidadesY, tiempoVelocidades,hiloCerrado

	nombreArchivo = sys.argv[1]
	f = np.genfromtxt("./src/Taller2/resources/{}".format(nombreArchivo),skip_header=1) #Obtencion de velocidades del archivo pasado por parametros
	f = np.transpose(f) #Transposicion de la matriz de datos f para clasificar los datos mas facilmente
	velocidadesX = f[1]
	velocidadesY = f[0]
	tiempoVelocidades = f[2]

	#Se inicializa un nodo con el nombre P3_SimuladorPosicionPioneer
	rospy.init_node('P3_SimuladorPosicionPioneer', anonymous=True)

	#Publicacion al topico de velocidades para los motores del robot pioneer
	pubMotorsVel = rospy.Publisher('motorsVel',Float32MultiArray,queue_size=10)

	#Suscripcion al topico pioneerPosition
	rospy.Subscriber("pioneerPosition",Twist,callbackPioneerPosition)
	rospy.Subscriber("simulationTime",Float32,callbackSimulationTime)

    #Tiempo durante el cual duerme el nodo
	rate = rospy.Rate(10)

	try:
		#Tiempo durante el cual duerme el nodo
		rate = rospy.Rate(10)
		hilo=threading.Thread(target= graficar, name='Graficar')
		hilo.start()
		while not rospy.is_shutdown():
			determinarVelocidadActual()
			#Publica la velocidad del robot en el topico motorsVel
			mensaje = Float32MultiArray(data=[velocidadActualM1,velocidadActualM2])
			pubMotorsVel.publish(mensaje) 

			#Se envia a dormir al nodo
			rate.sleep()
	except Exception as e:
		raise e

if __name__ == '__main__':
	main()
else:
	rospy.loginfo("Fallo al cargar codigo")