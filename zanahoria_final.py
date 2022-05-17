#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import time
from matplotlib import pyplot as plt 

class Follow_Carrot(object):

    def __init__(self):
        rospy.init_node('zanahoria_final')
        self.suscribirse()
        
        self.iniciar()

    def suscribirse(self): # Se conecta a los nodos
        # Turtlebot connections
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)

        # Conexxion PID
        self.objetivo = rospy.Publisher( '/robot_ang/setpoint', Float64, queue_size = 1 )
        self.state = rospy.Publisher( '/robot_ang/state', Float64, queue_size = 1 )
        self.recibir_PID = rospy.Subscriber('/robot_ang/control_effort', Float64, self.movimiento)

        #posicion robot
        self.recibir_odom = rospy.Subscriber( '/odom', Odometry, self.odometria)

        #Falta suscrbir a qn manda el objetivo
        self.recibir_objetivos = rospy.Subscriber( '/goal_list', Path, self.destinos)


    def movimiento(self, direccion): #Utiliza la suscripcion al controlador para estar moviendose
        #si no recibe nada del controlador no hace nada
        cambio = direccion.data
        speed = Twist()            
        speed.linear.x = 0.1
        speed.angular.z = cambio
        self.cmd_vel_mux_pub.publish(speed)


    def destinos(self, targets): #Crea la lista del path
        if len(self.objetivos) ==0:
            self.objetivos = []
            for i in targets.poses:
                target_x = i.pose.position.x
                target_y = i.pose.position.y
                self.objetivos.append([target_x, target_y])
            self.objetivos_graficos = self.objetivos
        self.llamar_controlador()



    def iniciar(self): #Inicia valores de la clase
        self.posicion = []
        frecuencia = 10 #0.5
        self.contador = 0
        self.rate_obj = rospy.Rate( frecuencia )
        self.objetivos = []
        self.initial_time = 0
        self.partiendo = 0
        self.zanahoria = -1
        self.error_medio = 0
        self.cuenta_total = 0
        pass

    def odometria(self, odom): #Gestiona la recepcion de la odometria
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w ) )
        self.posicion.append([x+1, y+1, yaw])


    def llamar_controlador(self): 
        while not rospy.is_shutdown():
            distancia_objetivo = self.calcular_distancia(self.posicion[-1], self.objetivos[-1])
            if distancia_objetivo>0.2: #Llama al controlador y las funciones necesarias si la distancia es mayor a 0.2
                self.rate_obj.sleep()
                self.actualizar_contador()
                self.angulo_objetivo = self.calcular_diferencia_angular()
                angulo_actual = self.posicion[-1][2]
                self.objetivo.publish(self.angulo_objetivo) #angulo_objetivo*180/np.pi 
                self.diferencia_angular = self.angulo_objetivo - angulo_actual
                self.state.publish(angulo_actual)
                #print(angulo_actual, self.angulo_objetivo, 'angulos87')
            else: #Osino llama al grafico
                self.graficar()
            

    def actualizar_contador(self): #Llama a funciones q buscan carrot point y punto minimo
        posicion_actual = self.posicion[-1]
        punto_minimo, location = self.encontrar_minimo(posicion_actual)
        zanahoria = self.zanahoria
        self.zanahoria = self.encontrar_zanahoria(punto_minimo, location)
        #if zanahoria != self.zanahoria:
            #print(punto_minimo, self.zanahoria, 'carrot95')
            #print(punto_minimo)
       

    def calcular_distancia(self, puntoA, puntoB): #Funcion para calcular distancia entre puntos
        distancia = ((puntoA[0]-puntoB[0])**2+ (puntoA[1]-puntoB[1])**2)**0.5
        return distancia

    def encontrar_minimo(self, post): #Encuentra la minima distancia entre el path y el robot y su ubicacion
        minimo = 1000
        punto_min = -1
        for i in range(len(self.objetivos)-1):
            pendiente = (self.objetivos[i+1][1]-self.objetivos[i][1])/(self.objetivos[i+1][0]-self.objetivos[i][0])
            c = self.objetivos[i+1][1]-pendiente*self.objetivos[i+1][0]
            distancia = abs((pendiente*post[0]-post[1]+c)/(pendiente**2+1)**0.5)
            if distancia<minimo:                
                if pendiente != 0:
                    i_pendiente = -1/pendiente
                    i_c = self.objetivos[i+1][1]-i_pendiente*self.objetivos[i+1][0]
                    x = (c-i_c)/(i_pendiente-pendiente)
                    y = pendiente*x+c
                else:
                    y = self.objetivos[i][1]
                    x = post[0]
                condicion_1 = self.objetivos[i][1]<=y<=self.objetivos[i+1][1]
                condicion_2 = self.objetivos[i][1]>=y>=self.objetivos[i+1][1]
                condicion_3 = self.objetivos[i][0]<=x<=self.objetivos[i+1][0]
                condicion_4 = self.objetivos[i][0]>=x>=self.objetivos[i+1][0]
                if (condicion_1 and (condicion_3 or condicion_4)) or (condicion_2 and (condicion_3 or condicion_4)):
                    minimo = distancia
                    punto_min = [x, y]
                    ubicacion = i
        self.error_medio += self.calcular_distancia(punto_min, [post[0],post[1]])
        self.cuenta_total += 1
        return punto_min, ubicacion
        

    def encontrar_zanahoria(self, punto, ubicacion): #Encuentra la zanahoria
        diferencia = 0.5 #0.7
        distancia = 0
        coordenadas_zanahoria = [0, 0]
        for i in range(ubicacion, len(self.objetivos)-1):
            if distancia<diferencia:
                #print(distancia, coordenadas_zanahoria, self.objetivos[i], i)
                coordenadas_zanahoria[0] = coordenadas_zanahoria[0]-self.objetivos[i][0]+self.objetivos[i+1][0]
                coordenadas_zanahoria[1] = coordenadas_zanahoria[1]-self.objetivos[i][1]+self.objetivos[i+1][1]
                distancia = self.calcular_distancia(coordenadas_zanahoria, [0, 0])
                final = i
                if distancia > diferencia:
                    #tamano = self.calcular_distancia(coordenadas_zanahoria, punto)
                    coordenadas_zanahoria[0] = diferencia*coordenadas_zanahoria[0]/distancia+punto[0]
                    coordenadas_zanahoria[1] = diferencia*coordenadas_zanahoria[1]/distancia+punto[1]
        distancia_fin = self.calcular_distancia(self.objetivos[len(self.objetivos)-1], punto)
        if ubicacion>5:
            self.objetivos = self.objetivos[5:]
        else:
            self.objetivos = self.objetivos[ubicacion:]
        if distancia_fin< diferencia:
            coordenadas_zanahoria = self.objetivos[len(self.objetivos)-1]
        return coordenadas_zanahoria
        
    def calcular_diferencia_angular(self): #Calcula angulo para control PID
        posicion_actual = self.posicion[-1]
        objetivo_actual = self.zanahoria
        x_obj = objetivo_actual[0] - posicion_actual[0]
        y_obj = objetivo_actual[1] - posicion_actual[1]
        if x_obj==0:
            if y_obj>0:
                valor = np.pi/2
            else:
                valor = -np.pi/2
        else:
            valor = np.arctan(y_obj/x_obj)
            if x_obj<0:
                if y_obj<=0:
                    valor -= np.pi
                else:
                    valor += np.pi
        '''if posicion_actual[1]>2:
            print('f161', posicion_actual, valor, objetivo_actual)'''
        return valor

    def graficar(self): #Grafica e imprime el error cuadratico
        self.longitud = []
        self.altura = []
        for i in self.posicion:
            self.longitud.append(i[0]) 
            self.altura.append(i[1])
        self.longitud_objetivos = []
        self.altura_objetivos = []
        for i in self.objetivos_graficos:
            self.longitud_objetivos.append(i[0]) 
            self.altura_objetivos.append(i[1])
        longitud = np.array(self.longitud)
        altura = np.array(self.altura)
        longitud_objetivos = np.array(self.longitud_objetivos)
        altura_objetivos = np.array(self.altura_objetivos)
        odom = plt.plot(longitud, altura, c = 'green') #genera el grafico del mov del robot
        esperado = plt.plot(longitud_objetivos, altura_objetivos, c= 'red') #genera el grafico del path
        print(self.error_medio/self.cuenta_total) #Termina de calcular el error cuadratico medio
        plt.show()








if __name__ == '__main__':
  follow_the_car = Follow_Carrot()
  print('corriendo')
  rospy.spin()
  





