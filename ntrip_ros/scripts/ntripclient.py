#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

# CAMBIO 1: Usamos el paquete estandar rtcm_msgs en lugar del pesado mavros_msgs
from rtcm_msgs.msg import Message 

import datetime
from httplib import HTTPConnection
from base64 import b64encode
from threading import Thread
import time # Anadido para los delays de reconexion

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def run(self):
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros_UMA', # Personalizado
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(self.ntc.ntrip_user + ':' + self.ntc.ntrip_pass)
        }
        
        # CAMBIO 2: Bucle infinito de reconexion para tolerancia a fallos
        while not self.stop and not rospy.is_shutdown():
            try:
                rospy.loginfo("Conectando al servidor NTRIP (RAP): " + self.ntc.ntrip_server)
                connection = HTTPConnection(self.ntc.ntrip_server, timeout=10)
                now = datetime.datetime.utcnow()
                
                # Enviamos la trama NMEA para decirle a la RAP donde estamos (Malaga)
                connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga % (now.hour, now.minute, now.second), headers)
                
                response = connection.getresponse()
                if response.status != 200: 
                    rospy.logwarn("Error de conexion NTRIP. Estado: {}. Reintentando en 5s...".format(response.status))
                    time.sleep(5)
                    continue

                rospy.loginfo("Conexion NTRIP establecida con exito.")
                
                # CAMBIO 3: Instanciamos el nuevo tipo de mensaje
                rmsg = Message() 
                
                while not self.stop and not rospy.is_shutdown():
                    data = response.read(1)
                    if not data: # Si el servidor cierra la conexion, salimos para reconectar
                        break
                    if data != chr(211): # 211 (0xD3) es el byte de inicio del estandar RTCM 3
                        continue
                    
                    l1 = ord(response.read(1))
                    l2 = ord(response.read(1))
                    pkt_len = ((l1 & 0x3) << 8) + l2
            
                    pkt = response.read(pkt_len)
                    parity = response.read(3)
                    
                    if len(pkt) != pkt_len:
                        rospy.logerr("Error de longitud en paquete RTCM: Esperado {}, Recibido {}".format(pkt_len, len(pkt)))
                        continue
                    
                    rmsg.header.seq += 1
                    rmsg.header.stamp = rospy.get_rostime()
                    
                    # CAMBIO 4: Forzamos el formato a LISTA para evitar el error de ROS (list or tuple type)
                    raw_data = data + chr(l1) + chr(l2) + pkt + parity
                    rmsg.message = list(bytearray(raw_data)) 
                    
                    self.ntc.pub.publish(rmsg)
                    
            except Exception as e:
                rospy.logerr("Excepcion en la conexion NTRIP: {}. Reintentando...".format(e))
                time.sleep(5)
            finally:
                try:
                    connection.close()
                except:
                    pass

class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic', '/rtcm') # Valor por defecto seguro
        self.nmea_topic = rospy.get_param('~nmea_topic', '/nmea_sentence')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.nmea_gga = rospy.get_param('~nmea_gga')

        # CAMBIO 5: Publicamos el nuevo tipo de mensaje
        self.pub = rospy.Publisher(self.rtcm_topic, Message, queue_size=10)

        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True
            self.connection.join() # Aseguramos que el hilo se cierra limpiamente

if __name__ == '__main__':
    c = ntripclient()
    c.run()