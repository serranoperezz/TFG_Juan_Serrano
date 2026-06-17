#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

WHEELBASE = 0.32        # distancia entre ejes del UMA-RACECAR (ajusta si es distinta)
MAX_STEERING = 0.4      # limite fisico del servo en radianes (~23 grados)

def callback(data):
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    v = data.linear.x
    wz = data.angular.z

    msg.drive.speed = v

    # Conversion velocidad angular -> angulo de volante (modelo Ackermann)
    if abs(v) > 0.01:
        steering = math.atan2(WHEELBASE * wz, v)
    else:
        steering = 0.0

    steering = max(-MAX_STEERING, min(MAX_STEERING, steering))
    msg.drive.steering_angle = steering

    pub.publish(msg)
    rospy.logdebug("Publicando: v=%f, angle=%f", v, steering)

rospy.init_node('cmd_vel_to_ackermann')
rospy.loginfo("Nodo 'cmd_vel_to_ackermann' iniciado. Esperando datos en /cmd_vel...")
pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)
rospy.Subscriber('/cmd_vel', Twist, callback)
rospy.spin()
