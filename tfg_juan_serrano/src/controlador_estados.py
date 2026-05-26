#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import sys, select, termios, tty

class DirectorOrquesta:
    def __init__(self):
        rospy.init_node('controlador_estados', anonymous=True)
        
        # Publicador del estado del coche
        self.estado_pub = rospy.Publisher('/modo_coche', String, queue_size=10)
        self.estado_actual = "IDLE"
        
        # Suscriptor al mando de PS3
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # Variables antibloqueo para los botones del mando (evita rebotes)
        self.btn_select_presionado = False
        self.btn_start_presionado = False
        
        # Guardar configuración de la terminal para el teclado
        self.settings = termios.tcgetattr(sys.stdin)

    def joy_callback(self, msg):
        # --- Boton SELECT (Botón 8): Iniciar/Parar GRABACIÓN ---
        if len(msg.buttons) > 8:
            if msg.buttons[8] == 1 and not self.btn_select_presionado:
                self.btn_select_presionado = True
                if self.estado_actual == "IDLE":
                    self.cambiar_estado("GRABANDO")
                elif self.estado_actual == "GRABANDO":
                    self.cambiar_estado("IDLE")
            elif msg.buttons[8] == 0:
                self.btn_select_presionado = False

        # --- Boton START (Botón 9): Iniciar/Parar CARRERA ---
        if len(msg.buttons) > 9:
            if msg.buttons[9] == 1 and not self.btn_start_presionado:
                self.btn_start_presionado = True
                if self.estado_actual == "IDLE":
                    self.cambiar_estado("CARRERA")
                elif self.estado_actual == "CARRERA":
                    self.cambiar_estado("IDLE")
            elif msg.buttons[9] == 0:
                self.btn_start_presionado = False

        # --- Boton FLECHA ABAJO (Botón 14): PARADA DE EMERGENCIA ---
        if len(msg.buttons) > 14:
            if msg.buttons[14] == 1:
                if self.estado_actual != "IDLE":
                    rospy.logwarn("¡BOTÓN DEL PÁNICO (FLECHA ABAJO)! Forzando IDLE...")
                    self.cambiar_estado("IDLE")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def cambiar_estado(self, nuevo_estado):
        if self.estado_actual != nuevo_estado:
            self.estado_actual = nuevo_estado
            self.imprimir_menu()

    def imprimir_menu(self):
        print("\n" + "="*50)
        print("   DIRECTOR DE ESTADOS (TFG Juan y Alba)")
        print("="*50)
        print(" ESTADO ACTUAL: [ " + self.estado_actual + " ]")
        print("-" * 50)
        print(" Controles Teclado | Controles Mando PS3")
        print("   'G'             |   SELECT            -> GRABAR")
        print("   'R'             |   START             -> CARRERA")
        print("    -              |   FLECHA ABAJO      -> PARADA EMERGENCIA")
        print("   'Q' o Ctrl+C    |      -              -> Apagar sistema")
        print("="*50 + "\n")

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        self.imprimir_menu()

        while not rospy.is_shutdown():
            tecla = self.getKey()

            # Logica del teclado
            if tecla.lower() == 'g':
                self.cambiar_estado("GRABANDO" if self.estado_actual == "IDLE" else "IDLE")
            elif tecla.lower() == 'r':
                self.cambiar_estado("CARRERA" if self.estado_actual == "IDLE" else "IDLE")
            elif tecla.lower() == 'q' or tecla == '\x03':
                break

            # Publicamos el estado constantemente
            self.estado_pub.publish(self.estado_actual)
            rate.sleep()

if __name__ == '__main__':
    try:
        director = DirectorOrquesta()
        director.run()
    except rospy.ROSInterruptException:
        pass