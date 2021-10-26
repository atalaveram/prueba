#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
class MovementDetector(object):
    def __init__(self):
        #Inicializamos los valores de la clase
        self._mved_distance=Float64()
        self._mved_distance.data=0.0
        self._current_position=Point()
        self.get_init_position()
        self.distance_mved_pub=rospy.Publisher('/moved_distance',Float64,queue_size=1)
        self.distance_mved_sub=rospy.Subscriber('/odom',Odometry,self.odom_callback)
    def get_init_position(self):
        data_odom=None
        while data_odom is None:
            try:
                data_odom=rospy.wait_for_message('/odom',timeout=1)
            except:
                rospy.loginfo("El topico odom no se encuentra activo, esperando")
        self._current_position.x=data_odom.pose.pose.position.x
        self._current_position.y=data_odom.pose.pose.position.y
        self._current_position.z=data_odom.pose.pose.position.z
    def odom_callback(self,msg):
        NewPos=msg.pose.pose.position
        self._mved_distance.data+=self.calculate_distance(NewPos,self._current_position)
        self.update_curpos(NewPos)
    def update_curpos(self,newpos):
        self._current_position.x=newpos.x
        self._current_position.y=newpos.y
        self._current_position.z=newpos.z
    def calculate_distance(self,newpos,curpos):
        x2=newpos.x
        x1=curpos.x
        y2=newpos.y
        y1=curpos.y
        dist=math.hypot(x2-x1,y2-y1)
        return dist
if __name__=='__main__':
    pass