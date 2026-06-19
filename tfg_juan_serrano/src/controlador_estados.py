#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from ublox_msgs.msg import NavPVT 
import sys, select, termios, tty
import os

class DirectorOrquesta:
    def __init__(self):
        rospy.init_node('controlador_estados', anonymous=True)
        
        # Publicador del modo de control del coche
        self.estado_pub = rospy.Publisher('/modo_coche', String, queue_size=10)
        self.estado_actual = "IDLE"
        
        # Telemetría del GPS (Precisión)
        self.h_acc = 99.9  
        self.v_acc = 99.9  
        self.rtk_status = "SIN SEÑAL"
        
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        rospy.Subscriber('/ublox_gps/navpvt', NavPVT, self.gps_pvt_callback)
        
        # Variables para control de flanco de los botones
        self.btn_select_presionado = False
        self.btn_start_presionado = False
        
        self.settings = termios.tcgetattr(sys.stdin)

    def gps_pvt_callback(self, msg):
        # Conversión de milímetros a metros
        self.h_acc = msg.hAcc / 1000.0
        self.v_acc = msg.vAcc / 1000.0
        
        # Estado del RTK según los flags del ZED-F9P
        flags = msg.flags
        carr_soln = (flags >> 6) & 0x03 
        
        if carr_soln == 2:
            self.rtk_status = "RTK FIX (Centimétrica ✔)"
        elif carr_soln == 1:
            self.rtk_status = "RTK FLOAT (Métrica ⚠)"
        else:
            self.rtk_status = "GPS ESTÁNDAR (3D Fix ✖)"

    def joy_callback(self, msg):
        # Botón SELECT (Índice 0): Alternar grabación
        if len(msg.buttons) > 0:
            if msg.buttons[0] == 1 and not self.btn_select_presionado:
                self.btn_select_presionado = True
                self.cambiar_estado("GRABANDO" if self.estado_actual == "IDLE" else "IDLE")
            elif msg.buttons[0] == 0:
                self.btn_select_presionado = False

        # Botón START (Índice 3): Alternar carrera
        if len(msg.buttons) > 3:
            if msg.buttons[3] == 1 and not self.btn_start_presionado:
                self.btn_start_presionado = True
                self.cambiar_estado("CARRERA" if self.estado_actual == "IDLE" else "IDLE")
            elif msg.buttons[3] == 0:
                self.btn_start_presionado = False

        # Botón FLECHA ABAJO (Índice 6): Parada de emergencia
        if len(msg.buttons) > 6:
            if msg.buttons[6] == 1:
                if self.estado_actual != "IDLE":
                    rospy.logwarn("¡BOTÓN DEL PÁNICO! Forzando IDLE...")
                    self.cambiar_estado("IDLE")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05) 
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def cambiar_estado(self, nuevo_estado):
        if self.estado_actual != nuevo_estado:
            self.estado_actual = nuevo_estado

    def imprimir_menu(self):
        os.system('clear')
        print("="*55)
        print("        DIRECTOR DE ESTADOS (TFG Juan y Alba)")
        print("="*55)
        print(" ESTADO ACTUAL: [ " + self.estado_actual + " ]")
        print("-" * 55)
        print(" TELEMETRÍA GPS RTK (ZED-F9P):")
        print("   Modo de Precisión:  " + self.rtk_status)
        print("   Precisión Horiz. (Error):  {:.3f} metros".format(self.h_acc))
        print("   Precisión Vert.  (Error):  {:.3f} metros".format(self.v_acc))
        print("-" * 55)
        print(" Controles Teclado | Controles Mando PS3")
        print("   'G'             |   SELECT        -> GRABAR")
        print("   'R'             |   START         -> CARRERA")
        print("   'Q' o Ctrl+C    |   FLECHA ABAJO  -> PARADA EMERGENCIA")
        print("="*55 + "\n")

    def run(self):
        rate = rospy.Rate(5) 
        
        while not rospy.is_shutdown():
            tecla = self.getKey()

            if tecla.lower() == 'g':
                self.cambiar_estado("GRABANDO" if self.estado_actual == "IDLE" else "IDLE")
            elif tecla.lower() == 'r':
                self.cambiar_estado("CARRERA" if self.estado_actual == "IDLE" else "IDLE")
            elif tecla.lower() == 'q' or tecla == '\x03':
                break

            self.imprimir_menu()
            
            self.estado_pub.publish(self.estado_actual)
            rate.sleep()

if __name__ == '__main__':
    try:
        director = DirectorOrquesta()
        director.run()
    except rospy.ROSInterruptException:
        pass
