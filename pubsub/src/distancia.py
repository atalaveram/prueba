#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from pubsub.msg import Estatus
dist_val=[]
def process_msg_callback(msg):
    #dx=round(msg.twist.twist.linear.x,2)
    #Debido a que nuestro robot es (2,0) no puede moverse sobre el eje y
    #dy=msg.twist.twist.linear.y
    #theta=round(msg.twist.twist.angular.z,2)
    rospy.loginfo('Cantidad de distancias: {}'.format(len(msg.ranges)))
    rospy.loginfo('Cantidad de distancias validas: {}'.format(len(dist_val)))
    for dist in msg.ranges:
        if(dist>msg.range_min and dist<msg.range_max):
            dist_val.append(dist)
    pubmsg.codigo=0
    pubmsg.estado='El objeto mas cercano esta a {:.3f} m y el mas lejano a {:.3f} m'.format(min(dist_val),max(dist_val))
    del dist_val[:]
    """if dx==0.0 and theta==0.0:
        pubmsg.codigo=0
        pubmsg.estado='Detenido'
    elif dx!=0.0 and theta==0.0:
        pubmsg.codigo=100
        pubmsg.estado='Solo vel lineal: {} m/s'.format(dx)
    elif dx==0.0 and theta!=0.0:
        pubmsg.codigo=200
        pubmsg.estado='Solo vel angular: {} rad/s'.format(theta)
    elif dx!=0.0 and theta!=0.0:
        pubmsg.codigo=300
        pubmsg.estado='Movimiento lineal: {} m/s y angular: {} rad/s'.format(dx,theta)
    else:
        pubmsg.codigo=1000
        pubmsg.estado='Error'"""
    pub.publish(pubmsg)
rospy.init_node('med_distancia')
sub=rospy.Subscriber('scan',LaserScan,process_msg_callback)
pub=rospy.Publisher('estatus',Estatus,queue_size=2)
rate=rospy.Rate(2)
pubmsg=Estatus()
rospy.spin()
# if __name__=="__main__":
#     init_monitor()