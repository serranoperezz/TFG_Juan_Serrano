#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Nodo de seguimiento de trayectoria (Pure Pursuit).

Carga una ruta de puntos (x, y) desde un CSV y guia al vehiculo publicando
comandos Ackermann. La orientacion del vehiculo se estima ajustando una
recta (PCA) a la ventana reciente de posiciones (x, y) de la odometria,
sin depender de la orientacion de la IMU/EKF. Mientras el modo CARRERA
esta activo, registra la trayectoria real recorrida en otro CSV
(trayectoria_rep.csv, en el mismo directorio que la ruta de referencia)
para poder compararla despues.
"""

import csv
import math
import os
from collections import deque

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class SeguidorPurePursuit(object):

    def __init__(self):
        rospy.init_node("seguidor_puntos", anonymous=True)

        # --- Parametros ---
        self.ruta_archivo = os.path.expanduser(rospy.get_param("~ruta_archivo", "~/trayectoria.csv"))
        self.ruta_repetida = os.path.join(os.path.dirname(self.ruta_archivo), "trayectoria_rep.csv")

        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered_map")
        self.wheelbase = rospy.get_param("~wheelbase", 0.325)
        self.lookahead_dist = rospy.get_param("~lookahead_dist", 2)
        self.v_velocidad = rospy.get_param("~v_velocidad", 1.2)
        self.max_steering = rospy.get_param("~max_steering", 0.4)
        self.distancia_meta = rospy.get_param("~distancia_meta", 0.45)
        self.ventana_busqueda = rospy.get_param("~ventana_busqueda", 15)
        self.invertir_direccion = rospy.get_param("~invertir_direccion", True)

        # Estimacion de orientacion a partir de posiciones (x, y):
        # ajuste de recta (PCA) sobre una ventana deslizante de muestras.
        self.tam_ventana_yaw = rospy.get_param("~tam_ventana_yaw", 20)
        self.dist_min_yaw = rospy.get_param("~dist_min_yaw", 0.25)

        # --- Estado interno ---
        self.camino = []
        self.estado_actual = "IDLE"
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.yaw_listo = False
        self.historial_posiciones = deque(maxlen=self.tam_ventana_yaw)
        self.indice_actual = 0
        self.primer_ciclo = True
        self.meta_alcanzada = False
        self.archivo_rep = None
        self.writer_rep = None

        # --- Topicos ---
        self.cmd_pub = rospy.Publisher(
            '/vesc/high_level/ackermann_cmd_mux/input/nav_1', AckermannDriveStamped, queue_size=10)
        self.marker_pub = rospy.Publisher("/visualizacion_camino", Marker, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher("/punto_objetivo", Marker, queue_size=1)

        rospy.Subscriber("/modo_coche", String, self.estado_callback, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)

        rospy.on_shutdown(self.cerrar_csv_repeticion)

        rospy.loginfo("PILOTO: Pure Pursuit listo. Ruta=%s | Repeticion=%s",
                      self.ruta_archivo, self.ruta_repetida)

    # ------------------------------------------------------------------
    # Gestion de estados
    # ------------------------------------------------------------------
    def estado_callback(self, msg):
        nuevo = msg.data.strip().upper()

        if nuevo == "CARRERA" and self.estado_actual != "CARRERA":
            if self.cargar_csv():
                self.indice_actual = 0
                self.primer_ciclo = True
                self.meta_alcanzada = False
                self.historial_posiciones.clear()
                self.yaw_listo = False
                self.publicar_camino_estatico()
                self.abrir_csv_repeticion()
                rospy.loginfo("PILOTO: Modo CARRERA activado. %d puntos cargados.", len(self.camino))
            else:
                rospy.logerr("PILOTO: Error al cargar el CSV.")
                nuevo = "ERROR"

        elif nuevo != "CARRERA" and self.estado_actual == "CARRERA":
            self.publicar_comando(0.0, 0.0)
            self.cerrar_csv_repeticion()
            rospy.loginfo("PILOTO: Modo CARRERA desactivado.")

        self.estado_actual = nuevo

    # ------------------------------------------------------------------
    # CSV: ruta de referencia y trayectoria repetida
    # ------------------------------------------------------------------
    def cargar_csv(self):
        self.camino = []
        if not os.path.exists(self.ruta_archivo):
            return False
        try:
            with open(self.ruta_archivo, "r") as archivo:
                reader = csv.reader(archivo)
                for row in reader:
                    if not row or not row[0].strip():
                        continue
                    try:
                        self.camino.append({"x": float(row[0]), "y": float(row[1])})
                    except ValueError:
                        continue
        except Exception as exc:
            rospy.logerr("PILOTO: Error leyendo CSV: %s", exc)
            return False
        return len(self.camino) > 0

    def abrir_csv_repeticion(self):
        self.cerrar_csv_repeticion()
        self.archivo_rep = open(self.ruta_repetida, "w")
        self.writer_rep = csv.writer(self.archivo_rep)

    def cerrar_csv_repeticion(self):
        if self.archivo_rep:
            self.archivo_rep.close()
        self.archivo_rep = None
        self.writer_rep = None

    def registrar_punto_repetido(self):
        if self.writer_rep:
            self.writer_rep.writerow([self.robot_x, self.robot_y])

    # ------------------------------------------------------------------
    # Odometria y estimacion de orientacion (a partir de x, y)
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        self.actualizar_orientacion()

        if self.estado_actual == "CARRERA":
            self.control_trayectoria()

    def actualizar_orientacion(self):
        """Estima robot_yaw como la direccion dominante de la trayectoria
        reciente, ajustando una recta (PCA) sobre los ultimos
        tam_ventana_yaw puntos de odometria. A diferencia de comparar solo
        dos muestras (con o sin umbral de distancia), una recta ajustada a
        toda la ventana no salta de forma discontinua de un ciclo a otro y
        promedia el ruido de posicion en vez de depender de un solo par
        de puntos."""
        self.historial_posiciones.append((self.robot_x, self.robot_y))

        if len(self.historial_posiciones) < self.tam_ventana_yaw:
            return  # ventana todavia no llena, se mantiene el ultimo yaw

        xs = [p[0] for p in self.historial_posiciones]
        ys = [p[1] for p in self.historial_posiciones]
        n = len(xs)
        mx = sum(xs) / n
        my = sum(ys) / n

        sxx = sum((x - mx) ** 2 for x in xs)
        syy = sum((y - my) ** 2 for y in ys)
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))

        extremo_x = xs[-1] - xs[0]
        extremo_y = ys[-1] - ys[0]
        if math.hypot(extremo_x, extremo_y) < self.dist_min_yaw:
            return  # ventana sin desplazamiento neto suficiente

        angulo = 0.5 * math.atan2(2.0 * sxy, sxx - syy)

        # La recta ajustada no distingue "hacia adelante" de "hacia atras":
        # se resuelve la ambiguedad con el sentido real de avance en la ventana.
        if math.cos(angulo) * extremo_x + math.sin(angulo) * extremo_y < 0:
            angulo += math.pi

        self.robot_yaw = math.atan2(math.sin(angulo), math.cos(angulo))
        self.yaw_listo = True

    # ------------------------------------------------------------------
    # Control de trayectoria (Pure Pursuit)
    # ------------------------------------------------------------------
    def control_trayectoria(self):
        if not self.camino or self.meta_alcanzada:
            return

        if not self.yaw_listo:
            self.publicar_comando(self.v_velocidad, 0.0)
            rospy.loginfo_throttle(0.5, "PILOTO: Calculando orientacion inicial (avance recto)...")
            return

        self.actualizar_indice_mas_cercano()

        dist_meta = self.distancia_a_punto(self.camino[-1])
        cerca_del_final = self.indice_actual >= len(self.camino) - 2
        if cerca_del_final and dist_meta <= self.distancia_meta:
            rospy.loginfo_once("PILOTO: Meta alcanzada.")
            self.publicar_comando(0.0, 0.0)
            self.meta_alcanzada = True
            return

        idx_obj = self.buscar_indice_lookahead(self.indice_actual)
        punto_obj = self.camino[idx_obj]

        self.publicar_target(punto_obj, 1.0, 0.0, 0.0)
        self.control_hacia_punto(punto_obj, self.v_velocidad, idx_obj)
        self.registrar_punto_repetido()

    def actualizar_indice_mas_cercano(self):
        # El primer ciclo arranca siempre en el punto 0 del CSV: evita que en
        # circuitos cerrados (inicio y final proximos) el coche salte
        # directamente al tramo final en lugar de recorrer la ruta completa.
        if self.primer_ciclo:
            self.indice_actual = 0
            self.primer_ciclo = False
            return

        inicio = self.indice_actual
        fin = min(self.indice_actual + self.ventana_busqueda, len(self.camino))

        mejor_idx = self.indice_actual
        mejor_dist = float("inf")
        for i in range(inicio, fin):
            d = self.distancia_a_punto(self.camino[i])
            if d < mejor_dist:
                mejor_dist = d
                mejor_idx = i

        self.indice_actual = mejor_idx

    def buscar_indice_lookahead(self, idx_inicio):
        limite = min(idx_inicio + 30, len(self.camino))
        for i in range(idx_inicio, limite):
            if self.distancia_a_punto(self.camino[i]) >= self.lookahead_dist:
                dx = self.camino[i]["x"] - self.robot_x
                dy = self.camino[i]["y"] - self.robot_y
                x_local = math.cos(self.robot_yaw) * dx + math.sin(self.robot_yaw) * dy
                if x_local > 0.05:
                    return i
        return min(idx_inicio + 5, len(self.camino) - 1)

    def control_hacia_punto(self, punto, velocidad, idx_obj):
        dx = punto["x"] - self.robot_x
        dy = punto["y"] - self.robot_y
        c = math.cos(self.robot_yaw)
        s = math.sin(self.robot_yaw)

        x_local = c * dx + s * dy
        y_local = -s * dx + c * dy
        dist2 = x_local * x_local + y_local * y_local

        if dist2 < 1e-6:
            self.publicar_comando(velocidad, 0.0)
            return

        steering = math.atan2(2.0 * self.wheelbase * y_local, dist2)
        steering = max(-self.max_steering, min(self.max_steering, steering))

        if self.invertir_direccion:
            steering = -steering

        rospy.loginfo_throttle(
            0.5,
            u"TRACCION | X:%.2f Y:%.2f Yaw:%.1fo | OBJ:[%d] | Y_LOCAL:%.2f | STEER:%.3f",
            self.robot_x, self.robot_y, math.degrees(self.robot_yaw),
            idx_obj, y_local, steering)

        self.publicar_comando(velocidad, steering)

    def distancia_a_punto(self, punto):
        return math.hypot(punto["x"] - self.robot_x, punto["y"] - self.robot_y)

    # ------------------------------------------------------------------
    # Publicacion (comandos y marcadores RViz)
    # ------------------------------------------------------------------
    def publicar_comando(self, velocidad, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.speed = velocidad
        msg.drive.steering_angle = steering
        self.cmd_pub.publish(msg)

    def publicar_camino_estatico(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.g = 1.0
        marker.color.a = 0.8
        for p in self.camino:
            marker.points.append(Point(p["x"], p["y"], 0.05))
        self.marker_pub.publish(marker)

    def publicar_target(self, punto, r, g, b):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.pose.position.x = punto["x"]
        marker.pose.position.y = punto["y"]
        marker.pose.position.z = 0.15
        self.target_pub.publish(marker)


if __name__ == "__main__":
    try:
        SeguidorPurePursuit()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
