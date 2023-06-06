#!/usr/bin/env python3

# Este nodo se encarga de computar las velocidades a las que se tienen que mover los motores para que el robot se mueva
# con una determinada velocidad, para ello:
#   -Se suscribe al topic cmd_vel
#   -Computa la cinemática inversa
#   -Publica las velocidades en el topic wheel_vels
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ow_interfaces.msg import WheelVels

import numpy as np


class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__("inversekinematics_node")
        self.vel_subscriber = self.create_subscription(
            Twist, "cmd_vel", self.callback_compute_vels, 10)
        self.wheel_vel_publisher = self.create_publisher(
            WheelVels,"wheel_vels",10)
    
    def callback_compute_vels(self, msg):
        angulos_ruedas = [np.pi/2,-np.pi/6, np.pi*7/6]
        omega = np.pi/2
        longitud = 0.290
        radio = 0.063

        M = np.array([[-np.sin(angulos_ruedas[0]+omega),    np.cos(angulos_ruedas[0]+omega),    longitud],
                       [-np.sin(angulos_ruedas[1]+omega),    np.cos(angulos_ruedas[1]+omega),    longitud],
                       [-np.sin(angulos_ruedas[2]+omega),    np.cos(angulos_ruedas[2]+omega),    longitud]]) * (1/radio)
        
        #Determinación de las velocidades angulares

        vels = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        
        w = np.matmul(M, vels)
        
        velocidad_ruedas = WheelVels()
        velocidad_ruedas.vel1 = float(w.tolist()[0])
        velocidad_ruedas.vel2 = float(w.tolist()[1])
        velocidad_ruedas.vel3 = float(w.tolist()[2])
        self.wheel_vel_publisher.publish(velocidad_ruedas)
        self.get_logger().info(np.array2string(w))

        return


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
