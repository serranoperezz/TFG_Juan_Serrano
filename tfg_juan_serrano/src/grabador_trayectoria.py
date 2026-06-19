#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import math
import os

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class Grabador(object):
    def __init__(self):
        rospy.init_node("grabador_trayectoria", anonymous=True)

        # Rutas de almacenamiento (Histórico estructurado y temporal de lectura rápida)
        self.directorio_base = os.path.expanduser("~/resultados_tfg_juan_serrano")
        self.ruta_temporal = os.path.expanduser("~/trayectoria.csv")

        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered_map")
        self.distancia_minima = rospy.get_param("~distancia_minima", 0.05)
        self.flush_cada_puntos = rospy.get_param("~flush_cada_puntos", 40)

        self.csv_file_historico = None
        self.csv_writer_historico = None
        self.csv_file_temporal = None
        self.csv_writer_temporal = None
        
        self.grabando = False
        self.last_x = None
        self.last_y = None
        self.num_puntos = 0

        # Suscriptores del nodo
        rospy.Subscriber("/modo_coche", String, self.estado_callback, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
        rospy.on_shutdown(self.cerrar_archivos)

        rospy.loginfo("NOTARIO: listo. Escuchando %s", self.odom_topic)

    def obtener_nueva_carpeta(self):
        if not os.path.exists(self.directorio_base):
            os.makedirs(self.directorio_base)

        # Escaneo y generación secuencial de directorios numéricos
        max_num = 0
        for item in os.listdir(self.directorio_base):
            ruta_item = os.path.join(self.directorio_base, item)
            if os.path.isdir(ruta_item):
                try:
                    num = int(item)
                    if num > max_num:
                        max_num = num
                except ValueError:
                    pass
        
        nueva_carpeta = os.path.join(self.directorio_base, str(max_num + 1))
        os.makedirs(nueva_carpeta)
        return nueva_carpeta

    def estado_callback(self, msg):
        estado = msg.data.strip().upper()
        if estado == "GRABANDO" and not self.grabando:
            self.abrir_archivos()
        elif estado != "GRABANDO" and self.grabando:
            self.cerrar_archivos()

    def abrir_archivos(self):
        carpeta_actual = self.obtener_nueva_carpeta()
        ruta_historico = os.path.join(carpeta_actual, "trayectoria.csv")

        # Inicialización de archivos CSV con cabeceras de coordenadas
        self.csv_file_historico = open(ruta_historico, "w")
        self.csv_writer_historico = csv.writer(self.csv_file_historico)
        self.csv_writer_historico.writerow(["x", "y"])

        self.csv_file_temporal = open(self.ruta_temporal, "w")
        self.csv_writer_temporal = csv.writer(self.csv_file_temporal)
        self.csv_writer_temporal.writerow(["x", "y"])

        self.grabando = True
        self.last_x = None
        self.last_y = None
        self.num_puntos = 0

        rospy.loginfo("NOTARIO: guardando en %s y en %s", ruta_historico, self.ruta_temporal)

    def odom_callback(self, msg):
        if not self.grabando or self.csv_writer_historico is None:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Filtro por umbral de distancia mínima recorrida
        if not self.debe_guardar(x, y):
            return

        x_str = "{:.6f}".format(x)
        y_str = "{:.6f}".format(y)

        self.csv_writer_historico.writerow([x_str, y_str])
        self.csv_writer_temporal.writerow([x_str, y_str])

        self.last_x = x
        self.last_y = y
        self.num_puntos += 1

        # Vaciado periódico del buffer a disco para mitigar pérdida de datos
        if self.flush_cada_puntos > 0 and self.num_puntos % self.flush_cada_puntos == 0:
            self.csv_file_historico.flush()
            self.csv_file_temporal.flush()

    def debe_guardar(self, x, y):
        if self.last_x is None or self.last_y is None:
            return True
        if self.distancia_minima <= 0.0:
            return True
        return math.hypot(x - self.last_x, y - self.last_y) >= self.distancia_minima

    def cerrar_archivos(self):
        # Cierre estructurado sincronizando los descriptores de archivos con el almacenamiento
        if self.csv_file_historico is not None:
            self.csv_file_historico.flush()
            os.fsync(self.csv_file_historico.fileno())
            self.csv_file_historico.close()
            self.csv_file_historico = None
            self.csv_writer_historico = None

        if self.csv_file_temporal is not None:
            self.csv_file_temporal.flush()
            os.fsync(self.csv_file_temporal.fileno())
            self.csv_file_temporal.close()
            self.csv_file_temporal = None
            self.csv_writer_temporal = None

        if self.grabando:
            rospy.loginfo("NOTARIO: grabacion terminada (%d puntos).", self.num_puntos)

        self.grabando = False

if __name__ == "__main__":
    try:
        Grabador()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
