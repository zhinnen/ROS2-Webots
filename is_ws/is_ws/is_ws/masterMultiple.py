import rclpy
import time
from math import cos, sin, tan, pi, atan2
import sys
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
import cv2
#import cv2.aruco as aruco
#from cv_bridge import CvBridge

class NavigateCells(Node):
    def __init__(self):
        super().__init__('NavigateCells_cmdvel')
        #self.create_timer(0.2, self.timer_callback)

        self.subs_right_ir = self.create_subscription(Range, 'right_DS', self.rightIR_cb, 1)
        self.subs_left_ir = self.create_subscription(Range, 'left_DS', self.leftIR_cb, 1)
        self.subs_front_ir = self.create_subscription(Range, 'front_DS', self.frontIR_cb, 1)
        self.subs_right_pr = self.create_subscription(Float64, 'right_PS', self.rightPR_cb, 1)
        self.subs_left_pr = self.create_subscription(Float64, 'left_PS', self.leftPR_cb, 1)
        self.subs_odom = self.create_subscription(Odometry, 'Odom', self.odom_callback, 1)
        
        self.get_logger().info("INITIALIZE")

        #publish cmd vel
        self.pubs_cmdvel = self.create_publisher(Twist, 'cmd_vel', 1)

        #vehicle parameters
        self.speed = 2.5
        self.angle_correction = 0.5

        #camera parameters
        self.numberOfPixels = 512
        self.reachThreshold = self.numberOfPixels * 0.4
        self.reached = False
        self.id = 0
        self.position = 0.0
        self.object1 = 0.0
        self.object2 = 0.0
        self.object3 = 0.0
        self.x = 0.0
        self.y = 0.0
        self.findCoordinates = 0
        self.travDistance = 0

        #camera
        self.camera_subsciber = self.create_subscription(Range, 'camera/raw_image', self.Image_processing_callback, 1)
        #self.bridge = CvBridge()

        #init parameters
        self.rds, self.lds, self.fds = 0.0, 0.0, 0.0
        self.lps, self.rps = 0, 0
        self.yaw = 0.0
        self.cmd = Twist()
        self.current_time = time.time()
        self.start_time = self.current_time
        self.cell_list = []
        self.currentCell = 0
        self.originalCell = 0
        self.n = 0
        self.count = 1
        self.stop = False
        self.command = "FIND"
        self.distance_trav = 0.0
        self.direction = ""
        self.start_position = 0.0
        self.degree = 0
        self.foundCell = 0
        self.get_logger().info("COMPLETE INITIALIZE")
        self.get_logger().info(self.command)

    def find(self):
        if(self.command == "FIND"):
            self.cmd.angular.z = 0.3
            self.cmd.linear.x = 0.0
            self.get_logger().info(str(self.yaw))
            if(self.object1 > 0 and self.object2 > 0 and self.object3 > 0 and self.yaw < 0.5 and self.yaw > -0.5):
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = 0.0
                self.command = "TRILAT"

            self.pubs_cmdvel.publish(self.cmd)

    
    def getDirection(self):
        if(self.yaw > 60 and self.yaw < 120):
            self.direction = "West"
        elif(self.yaw < -60 and self.yaw > -120):
            self.direction = "East"
        elif(self.yaw > 150 or self.yaw < -150):
            self.direction = "South"
        elif(self.yaw > -30 and self.yaw < 30):
            self.direction = "North"

    def CellsTraveled(self):
        if self.currentCell not in self.cell_list:
            self.cell_list.append(self.currentCell)
            self.count = self.count + 1
            self.get_logger().info(str(self.count))
            self.get_logger().info(str(self.cell_list))
        else:
            self.get_logger().info(str(self.cell_list))
    
    def turn(self):
        self.get_logger().info(str(self.yaw))
        if(self.command == "TURN"):
            self.cmd.angular.z = 0.3
            self.cmd.linear.x = 0.0
            if(self.degree-2 < self.yaw < self.degree+2):
                self.get_logger().info("reached desired degree")
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = 0.0
                self.command = "CHECKPATH"
            
            self.pubs_cmdvel.publish(self.cmd)

    def getCoordinates(self):
        if(self.command == "TRILAT"):
            self.triLat(-20,20,self.object1,-20,-20,self.object2,20,-20,self.object3)
            self.foundCell = 0
            self.currCell()
            self.command = "CHECKPATH"

    def triLat(self, x1,y1,r1,x2,y2,r2,x3,y3,r3):
        if(self.findCoordinates == 0):
            A = 2*x2 - 2*x1
            B = 2*y2 - 2*y1
            C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
            D = 2*x3 - 2*x2
            E = 2*y3 - 2*y2
            F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
            self.x = (C*E - F*B) / (E*A - B*D)
            self.y = (C*D - A*F) / (B*D - A*E)
            self.get_logger().info("X is: ")
            self.get_logger().info(str(self.x))
            self.get_logger().info("Y is: ")
            self.get_logger().info(str(self.y))
            self.findCoordinates = 1

    def currCell(self):
        if(self.foundCell == 0):
            if(self.x < -10 and self.y > 10):
                self.currentCell = 1
            if(self.x < 0 and self.y > 10):
                self.currentCell = 2
            if(self.x > 0 and self.y > 10):
                self.currentCell = 3
            if(self.x > 10 and self.y > 10):
                self.currentCell = 4
            if(self.x < -10 and self.y > 0):
                self.currentCell = 5
            if(self.x < 0 and self.y > 0):
                self.currentCell = 6
            if(self.x > 0 and self.x < 10 and self.y > 0):
                self.currentCell = 7
            if(self.x > 10 and self.y > 0):
                self.currentCell = 8
            if(self.x < -10 and self.y < 0 and self.y > -10):
                self.currentCell = 9
            if(self.x < 0 and self.x > -10 and self.y < 0 and self.y > -10):
                self.currentCell = 10
            if(self.x > 0 and self.x < 10 and self.y < 0 and self.y > -10):
                self.currentCell = 11
            if(self.x > 10 and self.y < 0 and self.y > -10):
                self.currentCell = 12
            if(self.x < -10  and self.y < -10):
                self.currentCell = 13
            if(self.x < 0 and self.x > -10 and self.y < -10):
                self.currentCell = 14
            if(self.x > 0 and self.x < 10 and self.y < -10):
                self.currentCell = 15
            if(self.x > 10 and self.y < -10):
                self.currentCell = 16

            self.cell_list.append(self.currentCell)
            self.foundCell = 1

    def checkWalls(self):
        if(self.command == "CHECKWALLS"):
            self.get_logger().info("CHECKING WALLS")
            inch_fds = (self.fds/0.0254)
            inch_rds = (self.rds/0.0254)
            inch_lds = (self.lds/0.0254)

            if(self.yaw > 2 and self.yaw < -2 and inch_fds > 10):
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = 0.0
                self.command = "CHECKPATH"
            elif(self.yaw < -88 and self.yaw > -92 and inch_fds > 10):
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = 0.0
                self.command = "CHECKPATH"
            elif(self.yaw > 88 and self.yaw < 92 and inch_fds > 10):
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = 0.0
                self.command = "CHECKPATH"
            elif(self.yaw > 178 or self.yaw < -178 and inch_fds > 10):
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = 0.0
                self.command = "CHECKPATH"
            
            self.cmd.angular.z = 0.3
            self.cmd.linear.x = 0.0

            self.pubs_cmdvel.publish(self.cmd)
    
    def drive(self):
        if(self.command == "DRIVE"):
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 1.25
            self.get_logger().info(str(self.distance_trav))
            inch_fds = (self.fds/0.0254)
            inch_rds = (self.rds/0.0254)
            inch_lds = (self.lds/0.0254)
            self.get_logger().info(str(inch_fds))

            if(self.travDistance == 0):
                self.start_position = inch_fds
                self.travDistance = 1

            self.distance_trav = self.start_position - inch_fds
            
            if(inch_fds < 10 and self.distance_trav < 1):
                self.command = "CHECKPATH"
                self.travDistance = 0

            self.getDirection()
            if(self.direction == "North"):
                if(self.distance_trav > 9.8 and self.count == 16):
                    self.command = "DONE"
                elif(self.distance_trav > 9.8):
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.0
                    self.y = self.y + self.distance_trav
                    self.travDistance = 0
                    self.command = "CHECKPATH"
            elif(self.direction == "East"):
                if(self.distance_trav > 9.8 and self.count == 16):
                    self.command = "DONE"
                elif(self.distance_trav > 9.8):
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.0
                    self.x = self.x + self.distance_trav
                    self.travDistance = 0
                    self.command = "CHECKPATH"
            elif(self.direction == "West"):
                if(self.distance_trav > 9.8 and self.count == 16):
                    self.command = "DONE"
                elif(self.distance_trav > 9.8):
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.0
                    self.x = self.x - self.distance_trav
                    self.travDistance = 0
                    self.command = "CHECKPATH"
            elif(self.direction == "South"):
                if(self.distance_trav > 9.8 and self.count == 16):
                    self.command = "DONE"
                elif(self.distance_trav > 9.8):
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.0
                    self.y = self.y - self.distance_trav
                    self.travDistance = 0
                    self.command = "CHECKPATH"

            
            self.pubs_cmdvel.publish(self.cmd)

    def checkPath(self):
        inch_fds = (self.fds/0.0254)
        if(inch_fds < 10):
            self.command = "CHECKWALLS"
            self.travDistance = 0
        if(self.command == "CHECKPATH"):
            self.getDirection()
            if(self.direction == "North"):
                self.originalCell = self.currentCell
                self.currentCell = self.currentCell - 4
                old_count = self.count
                self.CellsTraveled()
                if(self.count == old_count):
                    self.degree = 90
                    self.currentCell = self.originalCell
                    self.command = "TURN"
                else:
                    self.command = "DRIVE"
            elif(self.direction == "West"):
                self.originalCell = self.currentCell
                self.currentCell = self.currentCell - 1
                old_count = self.count
                self.CellsTraveled()
                if(self.count == old_count):
                    self.degree = 180
                    self.currentCell = self.originalCell
                    self.command = "TURN"
                else:
                    self.command = "DRIVE"
            elif(self.direction == "South"):
                self.originalCell = self.currentCell
                self.currentCell = self.currentCell + 4
                old_count = self.count
                self.CellsTraveled()
                if(self.count == old_count):
                    self.degree = -90
                    self.currentCell = self.originalCell
                    self.command = "TURN"
                else:
                    self.command = "DRIVE"
            elif(self.direction == "East"):
                self.originalCell = self.currentCell
                self.currentCell = self.currentCell + 1
                old_count = self.count
                self.CellsTraveled()
                if(self.count == old_count):
                    self.degree = 0
                    self.currentCell = self.originalCell
                    self.command = "TURN"
                else:
                    self.command = "DRIVE"

    def NavigatingModule(self):
        #self.get_logger().info(str(self.degree))
        #self.get_logger().info(str(self.yaw))
        #self.get_logger().info(self.direction)
        self.getDirection()
        if(self.count == 16 and self.command == "DONE"):
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 0.0
            self.pubs_cmdvel.publish(self.cmd)
        elif(self.command == "TURN"):
            self.turn()
        elif(self.command == "TRILAT"):
            self.getCoordinates()
        elif(self.command == "CHECKPATH"):
            self.checkPath()
        elif(self.command == "DRIVE"):
            self.drive()
        elif(self.command == "CHECKWALLS"):
            self.checkWalls()
        elif(self.command == "FIND"):
            self.find()

    def Image_processing_callback(self, msg):
        self.id = msg.radiation_type
        self.position = msg.range
        self.object1 = msg.field_of_view/0.0254
        self.object2 = msg.min_range/0.0254
        self.object3 = msg.max_range/0.0254

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * (180/pi)

    def rightIR_cb(self, msg):
        self.rds = msg.range
    
    def leftIR_cb(self, msg):
        self.lds = msg.range

    def rightPR_cb(self, msg):
        self.rps = msg.data

    def leftPR_cb(self, msg):
        self.ls = msg.data

    def frontIR_cb(self, msg):
        self.fds = msg.range
        self.NavigatingModule()  
            
def main(args=None):
    rclpy.init(args=args)
    NC = NavigateCells()
    rclpy.spin(NC)

    NC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()