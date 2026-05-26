#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def callback(data):
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    
    # Traducimos la velocidad lineal y el angulo de giro
    msg.drive.speed = data.linear.x
    msg.drive.steering_angle = data.angular.z
    
    pub.publish(msg)
    # Mensaje opcional para debug (puedes comentarlo si satura la consola)
    rospy.logdebug("Publicando: v=%f, angle=%f", data.linear.x, data.angular.z)

# Inicializamos el nodo
rospy.init_node('cmd_vel_to_ackermann')

# Mensaje de estado al iniciar
rospy.loginfo("Nodo 'cmd_vel_to_ackermann' iniciado. Esperando datos en /cmd_vel...")

pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
rospy.Subscriber('/cmd_vel', Twist, callback)

rospy.spin()