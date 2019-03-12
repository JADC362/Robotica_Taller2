#!/usr/bin/env python
import rospy 
from std_msgs.msg import Int32
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from pynput.keyboard import Key, Listener
import threading
import time
from Tkinter import *
import Tkinter
#--------------------------------Declaracion de todas las  variables--------------------------------

mat= Float32MultiArray()#matriz donde se almacenan los datos de la velocidad
velocidad=0 #vcelocidad global del robot
linear=Vector3() #vector donde se almacena la posicion lineal global
angular=Vector3() #vector donde se almacena la posicion angular global


vectorAcumuladoX=[]#vector donde se almacena todo el recorrido de el robot en X
vectorAcumuladoY=[]#vector donde se almacena todo el recorrido de el robot en Y
x=0 #posicion en x del robot
y=0 #posicion en y del robot
#----------------------------Declaracion de funciones-----------------------------------------------
def callbackPioneerPosition(msg): #funcion encargada de obtener la posicion del robot
	global linear, angular, vectorAcumuladoX, vectorAcumuladoY, x, y

	linear=msg.linear
	angular=msg.angular
	x = linear.x*10 
	y = linear.y*10 
	z = linear.z 
	vectorAcumuladoX.append(x)
	vectorAcumuladoY.append(y)

def ventanaDialogo(): #funcion encargada de lanzar la ventana de dialogo donde se introduce la velocidad
	global velocidad
	raiz = Tk()
	raiz.title("Entrada de velocidad")
	raiz.resizable(1,1)

	entrada=StringVar()

	
	Entrada=""


	frame1=Frame()
	frame1.pack()
	cuadroTexto=Entry(frame1, textvariable=entrada)
	cuadroTexto.grid(row=1, column=2, padx=10, pady=10)
	cuadroTexto.config(justify="right")
	nombreLabel= Label(frame1,text="velocidad:")
	nombreLabel.grid(row=1,column=1, padx=10, pady=10)

	botonEnviar =  Button(frame1, text="enviar velocidad", command=lambda:recibir())
	botonEnviar.grid(row=2, column=1, )

	def recibir():
		velocidad=int(entrada.get())
		entrada.set("")
		mat.data=[velocidad, velocidad]	
	
	raiz.mainloop()
			
def on_press(key): #Funcion al detectar una tecla presionada
	
	global velocidad
	try:		
		if key == key.up:
			mat.data=[mat.data[0],mat.data[0]*2]
		
		elif key == key.down:
			mat.data=[mat.data[1]*2,mat.data[1]]
	except Exception as e:
		rospy.loginfo("Presione las teclas arriba y abajo para girar")
def on_release(key): #Funcion al soltar una tecla
	
	try:
		if key == Key.esc: #Con ESC finaliza este thread
			return False		
		if key == key.up:
			mat.data=[mat.data[0], mat.data[0]]		
		if key == key.down:
			mat.data=[mat.data[1], mat.data[1]]
	except Exception as e:
		rospy.loginfo("Presione las teclas arriba y abajo para girar")

def ThreadInputs(): #Listener de Pynput en otro thread
		with Listener(on_press=on_press, on_release=on_release) as listener:
			listener.join()

def main():
	global iniciar, velocidad, hiloCerrado, x, y#variables globales
	#ejecucion del hilo donde se crea la ventana	
	hiloVentana=threading.Thread(target=ventanaDialogo)
	hiloVentana.daemon = True
	hiloVentana.start()
	#declaracion de las subscriptions y ublishers del nodo
	rospy.init_node('control', anonymous=True)
	pub1 = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)
	pub2 = rospy.Publisher('pauseSimulation', Bool ,queue_size=10)
	pub3 = rospy.Publisher('startSimulation', Bool ,queue_size=10)
	rospy.Subscriber("pioneerPosition",Twist,callbackPioneerPosition)
	
	rate = rospy.Rate(10)
	#inicializacion de las datos de la informacion que se le enviara al robot.
	mat.data=[0,0]
	mat.layout.dim.append(MultiArrayDimension())
	mat.layout.dim.append(MultiArrayDimension())
	mat.layout.dim[0].label = "leftMotor"
	mat.layout.dim[1].label = "rigthMotor"
	iniciar=True#inicia la simulacion
	#creacion y abertura de la imagen donde se plotea la posicion global del robot
	fig = plt.figure()
	plt.xlabel('xPos')
	plt.ylabel('yPos')
	plt.title('posicionGlobal')
	ax1 = fig.add_subplot(111)
	plt.grid()
	fig.show()
	#lanzamiendo del hilo encargado del teclado
	hiloTeclado=threading.Thread(target=ThreadInputs)
	hiloTeclado.daemon=True
	hiloTeclado.start()
	
	while not rospy.is_shutdown():
			time.sleep(0.01)
			pub1.publish(mat)#publicacion de las velocidades
			pub3.publish(iniciar)#enviar la orden de iniciar la simlacion		
			rate.sleep()
			ax1.scatter(x, y, c='b', edgecolors='b')#plotar la informacion en la grafica
			fig.canvas.draw()
			plt.savefig("Ejemplo1.jpg")#guardar figura		

if __name__ == '__main__':	
	main()


