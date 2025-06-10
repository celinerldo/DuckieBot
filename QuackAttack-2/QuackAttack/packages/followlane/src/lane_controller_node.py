#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from duckietown_msgs.msg import Twist2DStamped

class LaneController:
    def __init__(self):
        self.vehicle_name = rospy.get_param('~vehicle_name', 'duckiebot')
        self.sub = rospy.Subscriber(f'/{self.vehicle_name}/detect/lane', Float64, self.callback)
        self.pub = rospy.Publisher(f'/{self.vehicle_name}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

    def callback(self, msg):
        lane_center = msg.data
        print("Received lane center:", lane_center)
        
        # Paramètres de contrôle (tu peux ajuster)
        Kp = 0.015  # gain proportionnel
        v = 0.25  # vitesse constante
        
        

        if lane_center == -1: #renvoie -1 en cas d'absence de ligne
            omega = 0.0 #ligne non détectée, avance tout droit
            error = 0.0
        else:
            error = lane_center - 50
            omega = -Kp * error # Erreur par rapport au centre de l’image (50 est le milieu)

        # Commande moteur
        cmd = Twist2DStamped()
        cmd.v = v
        cmd.omega = omega

        self.pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('lane_controller_node')
    node = LaneController()
    rospy.spin()
