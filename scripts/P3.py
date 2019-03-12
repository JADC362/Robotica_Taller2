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

#Vectores que representan la posicion del robot
linear = Vector3()
angular = Vector3()

#Variable booleana que mantiene el hilo que grafica activo mientras no se cierre ninguna grafica 
hiloCerrado = False
hilo = 0

#Especificaciones del chasis del robot
length = 0.445 
width = 0.393 

#Especificaciones de las llantas del robot
diameter = 0.19532
radio = diameter/2 
WheelsWidth = 0.0475

#Distancia del punto P puesto en el chasis a el punto de contacto de las llantas 
l = (width/2)-WheelsWidth/2

#Parametros alpha y beta de las llantas
alpha1 = -np.pi/2
alpha2 = np.pi/2
beta1 = np.pi
beta2 = 0 
tetha = 0

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
simulationTimeAnterior = 0 #Tiempo anterior
pasoDeSimulacion = simulationTime-simulationTimeAnterior #Paso de simulacion utilizado por vrep
deltaTiempoVelocidades = simulationTime-simulationTimeB #Tiempo que lleva ejecutandose la velocidad actual 

#Contador de velocidad actual
contadorVelAc = 0

#Vectores de posicion actual del robot simulado
posicionXActualSimulada = []
posicionYActualSimulada = []

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

	global linear,angular,tetha,posicionInicialX,posicionInicialY
	linear = msg.linear 
	angular = msg.angular
	x = linear.x 
	y = linear.y
	tetha = angular.z
	posicionXActualSimulada.append(x)
	posicionYActualSimulada.append(y)

	if len(posicionXActualSimulada)==1:

		posicionXActualExacta[0] = posicionXActualSimulada[0]
		posicionYActualExacta[0] = posicionYActualSimulada[0]

	if len(posicionXActualSimulada) > 2:
		calcularRecorridoExacto()


#Funcion callback llamada cuando el topico simulationTime es actualizado
#La funcion se encarga de actualizar el tiempo actual, el tiempo anterior, el paso de simulacion y el tiempo que lleva la velocidad actual en el robot
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
	

#Funcion que grafica la posicion actual del pioneer y su correspondiente error (simulado vs teorico)
def graficar():
	global hiloCerrado,hilo
	plt.ion()
	wfig = 20
	hfig = 20
	proporcionTitulo = 0.9
	proporcionLabels = 0.8
	fig = plt.figure(figsize=(wfig,hfig))
	gs = gridspec.GridSpec(1, 2)
	gs.update(hspace=10);
	while not hiloCerrado:

		try: 
			plt.clf()
			ax0 = fig.add_subplot(gs[:, 0])
			ax0.plot(posicionXActualSimulada,posicionYActualSimulada,'m--', linewidth = 2)
			ax0.plot(posicionXActualExacta,posicionYActualExacta,'r:', linewidth = 2)
			plt.legend(('Posicion del robot simulado','Posicion del robot teorico'), fontsize=wfig*proporcionLabels)
			ax0.scatter(posicionXActualSimulada[len(posicionXActualSimulada)-1],posicionYActualSimulada[len(posicionYActualSimulada)-1], s=20**2, c= 250, alpha = 0.6 )
			ax0.set_xlabel("Posicion en el eje X del robot con respecto al marco global [m]",fontsize = wfig*proporcionLabels)
			ax0.set_ylabel("Posicion en el eje Y del robot con respecto al marco global [m]", fontsize = wfig*proporcionLabels)
			plt.title("Posicion simulada en tiempo real del robot", fontsize = wfig*proporcionTitulo)
			ax0.grid(True)
			ax0.scatter(posicionXActualExacta[len(posicionXActualExacta)-1],posicionYActualExacta[len(posicionYActualExacta)-1], s=10**2, c= 500, alpha = 0.4 )
			fig.savefig(os.getcwd()+"/src/Taller2/results/GraficasPunto3.png")
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

	global posicionXActualExacta,posicionYActualExacta, Error,radio,velocidadActualM1,velocidadActualM2, J1
	try:
		Rinv = np.array([[np.cos(tetha), -np.sin(tetha), 0],
		[np.sin(tetha), np.cos(tetha),0],[0 ,0 ,1]])
		phi = [[radio*velocidadActualM1],[radio*velocidadActualM2],[0]] #Vector phi que lleva las velocidades de los motores
		J1inv = np.linalg.inv(J1)
		velocidadesGlobales = Rinv.dot(J1inv.dot(np.array(phi))) #Velocidades lineal y angular actual del robot

		#Calculo de la posicion actual
		xExacta = (velocidadesGlobales[0]*pasoDeSimulacion+posicionXActualExacta[len(posicionXActualExacta)-1])
		yExacta = (velocidadesGlobales[1]*pasoDeSimulacion+posicionYActualExacta[len(posicionYActualExacta)-1])

		posicionXActualExacta.append(xExacta)
		posicionYActualExacta.append(yExacta)

		#Calculo del error 
		xSimulada = posicionXActualSimulada[len(posicionXActualSimulada)-1]
		ySimulada = posicionYActualSimulada[len(posicionYActualSimulada)-1]
		ErrorTotal = np.sqrt((xSimulada-xExacta)**2+(ySimulada-yExacta)**2)
		Error.append(ErrorTotal)
		tiempo.append(simulationTime)

	except Exception as e:
		pass



#Funcion principal del programa
def main():

	global velocidadesX, velocidadesY, tiempoVelocidades,hiloCerrado

	nombreArchivo = sys.argv[1]
	f = np.genfromtxt("./src/Taller2/resources/{}".format(nombreArchivo),skip_header=1) #Obtencion de velocidades del archivo pasado por parametros
	f = np.transpose(f) #Transposicion de la matriz de datos f para clasificar los datos mas facilmente
	velocidadesX = f[0]
	velocidadesY = f[1]
	tiempoVelocidades = f[2]

	#Se inicializa un nodo con el nombre P3_SimuladorPosicionPioneer
	rospy.init_node('P3_SimuladorPosicionPioneer', anonymous=True)

	#Publicacion al topico de velocidades para los motores del robot pioneer
	pubMotorsVel = rospy.Publisher('motorsVel',Float32MultiArray,queue_size=10)

	#Suscripcion al topico pioneerPosition
	rospy.Subscriber("pioneerPosition",Twist,callbackPioneerPosition)
	rospy.Subscriber("simulationTime",Float32,callbackSimulationTime)

	try:
		#Tiempo durante el cual duerme el nodo
		rate = rospy.Rate(10)
		hilo=threading.Thread(target= graficar, name='Graficar')
		hilo.start()
		time.sleep(0.1)
		while not rospy.is_shutdown():
			determinarVelocidadActual()
			#Publica la velocidad del robot en el topico motorsVel
			mensaje = Float32MultiArray(data=[velocidadActualM2,velocidadActualM1])
			pubMotorsVel.publish(mensaje) 

			#Se envia a dormir al nodo
			rate.sleep()
	except Exception as e:
		raise e
		pass

if __name__ == '__main__':
	main()
else:
	rospy.loginfo("Fallo al cargar codigo")