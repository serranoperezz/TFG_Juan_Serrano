#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import tf
import math
import os
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class GrabadorInteligente:
    def __init__(self):
        rospy.init_node('grabador_trayectoria', anonymous=True)
        self.ruta_archivo = os.path.expanduser('~/trayectoria.csv')
        self.csv_file = None
        self.csv_writer = None
        self.grabando = False
        self.last_x, self.last_y = -999.0, -999.0
        self.distancia_minima = 0.025 

        rospy.Subscriber('/modo_coche', String, self.estado_callback)
        rospy.Subscriber('/odometry/filtered_map', Odometry, self.odom_callback)
        rospy.loginfo("NOTARIO: Listo. Esperando 'GRABANDO'...")

    def estado_callback(self, msg):
        if msg.data == "GRABANDO" and not self.grabando:
            self.iniciar_grabacion()
        elif msg.data != "GRABANDO" and self.grabando:
            self.detener_grabacion()

    def iniciar_grabacion(self):
        self.csv_file = open(self.ruta_archivo, 'w') 
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'yaw', 'v', 'w'])
        self.csv_file.flush()
        self.last_x, self.last_y = -999.0, -999.0
        self.grabando = True
        rospy.loginfo("NOTARIO: Grabando en ~/trayectoria.csv")

    def detener_grabacion(self):
        if self.csv_file:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
            self.csv_file = None
        self.grabando = False
        rospy.loginfo("NOTARIO: Archivo guardado.")

    def odom_callback(self, msg):
        if not self.grabando: return 
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2) >= self.distancia_minima:
            q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            yaw = tf.transformations.euler_from_quaternion(q)[2]
            self.csv_writer.writerow([x, y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z])
            self.csv_file.flush()
            self.last_x, self.last_y = x, y

if __name__ == '__main__':
    GrabadorInteligente()
    rospy.spin()