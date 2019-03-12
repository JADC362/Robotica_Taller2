# taller2_5
CONTEXTO

Este es un repositorio creado a manera de contener la solución implementada a la tarea 2 del programa ROBÓTICA de la Universidad de los Andes. Esta se enfoca en la creación de un paquete de ROS que solucione 4 puntos diferentes. Estos 4 puntos se dirigen en la solución de retos respecto al control directo e inverso de un robot direccional, así como del entendimiento de la teoría detras de este.

DESCRIPCIÓN

Esta es un nodo que implementa soluciones a retos planteados con respecto al control de un robot direccional. Especificamente, el primer punto constanta el desarrollo de un punto teorico, mientas los siguientes tres constantan la solución de problematica respecto a este robot. Dicho robot se simula en el entorno de V-REP (http://www.coppeliarobotics.com/), programa que funciona bajo ROS (http://www.ros.org/) y por el cual se realiza la simulacin de robots estaticos y dinamicos. En este caso se tiene un robot Pioneer V3 (https://www.generationrobots.com/en/402395-robot-mobile-pioneer-3-dx.html), simulado en el entorno de VREP. Así, en los tres diferentes puntos se implementan soluciones que permiten controlar al robot con teclas y una interfaz, permitir graficar la curva que toma el robot de forma teórica y simulada y, por último, determinar una ley de control que controle el robot de forma proporcional para llegar a un punto dado.

REQUERIMIENTOS DE SISTEMA

	- Ubuntu 16.04 64 bits
	- ROS Kinetic
	- Qt5 Development Libraries
	- OpenGL
  	- V-REP PRO EDU

VERSION

	- Rosdistro: kinetic
	- Rosversion: 1.12.14
	- taller2_5: 1.0.0
  	- V-rep V: 3.6.0
	
VERSION LIBRERIAS PYTHON

	- rospy: 1.12.14
	- pynput: 1.4
  	- numpy: 1.15.1
	- matplotlib: 2.2.3
LIBRERIAS PYTHON BASE

	- time
	- os
	- sys
	- threading
	- TKinter
INSTALACIÓN

	1) Instalar primero ROS, siguiendo el tutorial alojado en la pagina http://wiki.ros.org/kinetic/Installation/Ubuntu y crear un workspace
	2) Descargar V-Rep Pro Edu de la pagina http://www.coppeliarobotics.com/downloads.html y alojar este en una carpeta como /Documentos
	3) Descargar el paquete taller2_5 del repositorio actual (https://github.com/JADC362/taller2_5) y almacenarlo en el workspace de ROS. 
				
COMPILACIÓN

	- cd ~/catkin_ws (o dirijirse al workspace creado)
	- source devel/setup.bash
	- catkin_make
PERMISOS

	Cada código creado debe darsele la opción de ejecutarse. Para este se implementa el siguiente codigo:
	- cd ~/catkin_ws/src/taller2_5/scripts/
	- chmod +x *.py 

EJECUCIÓN

	Lo primero es ejecutar el entorno de ros y luego ejecutar vrep en la escena dispuesta en este repositorio (Escena.ttt)
	- Abrir una nueva terminal
	- Correr: roscore
	- Abrir una nueva terminal
  	- Dirigirse al directorio de vrep y correr el programa como: ./vrep.sh
  	- Abrir una nueva terminal
	- source devel/setup.bash
	Despues consiste en ejecutar el nodo correspondiente al punto deseado. Para cada punto el codigo es el siguiente:
	1) rosrun taller2_5 P2.py
  	2) rosrun taller2_5 P3.py nombreArchivoPerfilVelocidadesconExtención
  	3) rosrun taller2_5 P4.py [xFinal] [yFinal] [θfinal]
	
CREADORES

	- John Alejandro Duarte Carraco
	- Jonathan Steven Roncancio Pinzon
	- Santiago Devia Valderrama
	- Miguel Angel Mozo Reyes
