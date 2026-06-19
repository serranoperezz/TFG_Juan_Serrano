#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from rtcm_msgs.msg import Message 

import datetime
from httplib import HTTPConnection
from base64 import b64encode
from threading import Thread
import time

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def run(self):
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros_UMA',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(self.ntc.ntrip_user + ':' + self.ntc.ntrip_pass)
        }
        
        # Bucle principal de conexión con tolerancia a fallos
        while not self.stop and not rospy.is_shutdown():
            try:
                rospy.loginfo("Conectando al servidor NTRIP: " + self.ntc.ntrip_server)
                connection = HTTPConnection(self.ntc.ntrip_server, timeout=10)
                now = datetime.datetime.utcnow()
                
                # Inyección de trama NMEA GGA para inicialización del stream VRS/PRS
                connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga % (now.hour, now.minute, now.second), headers)
                
                response = connection.getresponse()
                if response.status != 200: 
                    rospy.logwarn("Error de conexion NTRIP (Status: {}). Reintentando en 5s...".format(response.status))
                    time.sleep(5)
                    continue

                rospy.loginfo("Conexion NTRIP establecida con exito.")
                rmsg = Message() 
                
                # Desencapsulado del stream RTCM
                while not self.stop and not rospy.is_shutdown():
                    data = response.read(1)
                    if not data:
                        break
                    
                    # Sincronización: Preámbulo estándar RTCM 3 (0xD3 / 211)
                    if data != chr(211): 
                        continue
                    
                    # Lectura de longitud del mensaje (10 bits corregidos por máscara)
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
                    
                    # Serialización del frame RTCM completo a tipo de dato compatible con ROS
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

        # Carga de parámetros del servidor y configuración de tópicos
        self.rtcm_topic = rospy.get_param('~rtcm_topic', '/rtcm') 
        self.nmea_topic = rospy.get_param('~nmea_topic', '/nmea_sentence')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.nmea_gga = rospy.get_param('~nmea_gga')

        self.pub = rospy.Publisher(self.rtcm_topic, Message, queue_size=10)

        # Inicialización del hilo de comunicaciones
        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True
            self.connection.join()

if __name__ == '__main__':
    c = ntripclient()
    c.run()
