#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String

class SeguidorFiel:
    def __init__(self):
        rospy.init_node('seguidor_puntos', anonymous=True)
        self.ruta_archivo = os.path.expanduser('~/trayectoria.csv')
        self.camino = []
        
        # --- PARAMETROS DE CONTROL ---
        self.v_velocidad = 1.2       # Velocidad constante
        self.offset_puntos = 25      # Puntos por delante (aumentar si oscila)
        self.ganancia_giro = 3.2     # Sensibilidad del volante
        self.distancia_meta = 0.3    # Distancia para frenar al final

        self.estado_actual = "IDLE"
        self.robot_x = self.robot_y = self.robot_yaw = 0.0
        self.indice_actual = 0
        self.meta_alcanzada = False

        # Publicadores
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualizacion_camino', Marker, queue_size=1)
        
        # Suscriptores
        rospy.Subscriber('/modo_coche', String, self.estado_callback)
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.odom_callback)
        
        rospy.loginfo(" Sistema de Seguimiento de trayectoria activado.")

    def estado_callback(self, msg):
        nuevo = msg.data
        if nuevo == "CARRERA" and self.estado_actual != "CARRERA":
            if self.cargar_csv():
                self.indice_actual = 0
                self.meta_alcanzada = False
                self.estado_actual = "CARRERA"
                rospy.loginfo("¡BANDERA VERDE!")
            else:
                rospy.logerr("No se encontro el archivo de trayectoria.")
        else:
            # Si cambiamos a cualquier otro modo, el estado se actualiza 
            # y el bucle de odom_callback se encargara de frenar.
            self.estado_actual = nuevo

    def cargar_csv(self):
        self.camino = []
        if not os.path.exists(self.ruta_archivo): return False
        try:
            with open(self.ruta_archivo, 'r') as f:
                reader = csv.reader(f)
                next(reader) # Saltar cabecera
                for row in reader:
                    self.camino.append({'x': float(row[0]), 'y': float(row[1])})
            return len(self.camino) > 0
        except: return False

    def odom_callback(self, msg):
        # 1. Actualizar Pose
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.robot_yaw = tf.transformations.euler_from_quaternion(q)[2]
        
        # 2. Creamos el mensaje de comando vacio (por defecto todo a 0.0)
        cmd = Twist()

        # 3. Solo si estamos en carrera y no hemos llegado, calculamos velocidad
        if self.estado_actual == "CARRERA" and not self.meta_alcanzada:
            self.calcular_control(cmd)
            self.publicar_rviz()
        else:
            # SEGURIDAD: Si no estamos en carrera, nos aseguramos de que el comando sea cero
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # 4. PUBLICACION CONSTANTE: Sea cual sea el modo, enviamos el comando.
        # Esto evita que el coche "siga recto" al quitar el modo carrera.
        self.cmd_pub.publish(cmd)

    def calcular_control(self, cmd_msg):
        # Buscar el punto mas cercano para actualizar nuestro progreso
        dist_min = float('inf')
        for i in range(self.indice_actual, len(self.camino)):
            d = math.sqrt((self.camino[i]['x']-self.robot_x)**2 + (self.camino[i]['y']-self.robot_y)**2)
            if d < dist_min:
                dist_min = d
                self.indice_actual = i
            elif d > dist_min + 0.5: break

        # Punto objetivo con offset
        idx_obj = min(self.indice_actual + self.offset_puntos, len(self.camino) - 1)
        punto_obj = self.camino[idx_obj]

        # Comprobar Meta
        dist_meta = math.sqrt((self.camino[-1]['x']-self.robot_x)**2 + (self.camino[-1]['y']-self.robot_y)**2)
        if self.indice_actual >= len(self.camino) - 5 and dist_meta < self.distancia_meta:
            rospy.loginfo("PILOTO: Meta alcanzada.")
            self.meta_alcanzada = True
            return

        # Geometria de giro
        inc_x = punto_obj['x'] - self.robot_x
        inc_y = punto_obj['y'] - self.robot_y
        angulo_al_punto = math.atan2(inc_y, inc_x)
        error_yaw = math.atan2(math.sin(angulo_al_punto - self.robot_yaw), math.cos(angulo_al_punto - self.robot_yaw))

        # Esto reduce la velocidad si el error de giro es muy grande (curva cerrada)
        v_final = self.v_velocidad * (1.0 - min(abs(error_yaw), 0.5))
        
        # Rellenamos el mensaje que nos han pasado
        cmd_msg.linear.x = v_final
        cmd_msg.angular.z = error_yaw * self.ganancia_giro

    def publicar_rviz(self):
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = rospy.Time.now()
        m.type = Marker.LINE_STRIP; m.scale.x = 0.05
        m.color.g, m.color.a = 1.0, 0.8
        for p in self.camino:
            m.points.append(Point(p['x'], p['y'], 0.05))
        self.marker_pub.publish(m)

if __name__ == '__main__':
    try:
        SeguidorFiel(); rospy.spin()
    except rospy.ROSInterruptException: pass