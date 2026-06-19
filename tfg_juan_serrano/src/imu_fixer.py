#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

def callback(data):
    # Signo de la velocidad angular en Z invertido
    data.angular_velocity.z *= -1
    pub.publish(data)

rospy.init_node('imu_fixer', anonymous=True)
pub = rospy.Publisher('/imu_corregida', Imu, queue_size=10)
rospy.Subscriber('/imu', Imu, callback)
rospy.spin()
