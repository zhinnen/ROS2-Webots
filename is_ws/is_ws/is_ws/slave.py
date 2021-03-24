import rclpy
from rclpy.time import Time
from rclpy.clock import ROSClock
from datetime import datetime
from webots_ros2_core.webots_node import WebotsNode
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import numpy as np

import math
from math import sin, cos, tan, pi
import time

#isolate the front distacne sensor to determine what issue there is. Just for the distance sensor info. Check for throttle.
#look at publisher and subcriber and publish and webots. Look at the timer

#DEVICE_CONFIG = { 'camera1' : {'topic_name' : 'camera', 'timestep': 16} }

class service_node_vel(WebotsNode):
    def __init__(self, args):
        super().__init__("slave_node", args)

        #enable camera objects
        self.object1 = 0.0
        self.object2 = 0.0
        self.object3 = 0.0
        self.id1 = 0
        self.id2 = 0
        self.id3 = 0
        self.count = 0

        #enable sensors and timesteps
        self.timestep = 16
        self.cam_timestep = .034
        self.sensor_timer = self.create_timer(
            0.001 * self.timestep, self.sensor_callback)

        #sensor section        
        self.rightPosition_sensor = self.robot.getDevice('right wheel sensor')
        self.leftPosition_sensor = self.robot.getDevice('left wheel sensor')
        self.rightPosition_sensor.enable(self.timestep)
        self.leftPosition_sensor.enable(self.timestep)
        self.sensorPublisher_rightPosition = self.create_publisher(Float64, 'right_PS', 1)
        self.sensorPublisher_leftPosition = self.create_publisher(Float64, 'left_PS', 1)

        self.frontDistance_sensor = self.robot.getDevice('front_ds')
        self.rightDistance_sensor = self.robot.getDevice('right_ds')
        self.leftDistance_sensor = self.robot.getDevice('left_ds')
        self.frontDistance_sensor.enable(self.timestep)
        self.rightDistance_sensor.enable(self.timestep)
        self.leftDistance_sensor.enable(self.timestep)
        self.sensorPublisher_frontDistance = self.create_publisher(Range, 'front_DS', 1)
        self.sensorPublisher_rightDistance = self.create_publisher(Range, 'right_DS', 1)
        self.sensorPublisher_leftDistance = self.create_publisher(Range, 'left_DS', 1)

        #camera
        #self.start_device_manager(DEVICE_CONFIG)
        self.camera = self.robot.getDevice('camera1')
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        self.camera_publisher = self.create_publisher(Range, 'camera/raw_image', 1)
        self.cam_timer = self.create_timer(self.cam_timestep, self.camera_callback)

        #wheels section
        self.leftMotor = self.robot.getDevice('left wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0)

        self.rightMotor = self.robot.getDevice('right wheel motor')
        self.rightMotor.setPosition(float('inf'))
        self.rightMotor.setVelocity(0)

        self.motorMaxSpeed = self.leftMotor.getMaxVelocity()
        self.cmdVelSubscriber = self.create_subscription(Twist, 'cmd_vel', self.cmdVel_callback, 1)

        #Odometry data
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.time_step = 0.032
        self.prev_angle = 0.0
        self.prev_left_wheel_ticks = 0.0
        self.prev_right_wheel_ticks = 0.0
        self.last_time = 0.0
        self.wheelGap = 2.28
        self.wheelRadius = 0.8

        #Create Odom Publisher
        self.odom_pub = self.create_publisher(Odometry, 'Odom', 1)
        self.odom_timer = self.create_timer(self.time_step, self.odom_callback)
        self.get_logger().info('Sensor enabled')

    def cmdVel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

        leftSpeed = ((2.0 * msg.linear.x - msg.angular.z *
                       self.wheelGap) / (2.0 * self.wheelRadius))
        rightSpeed = ((2.0 * msg.linear.x + msg.angular.z *
                        self.wheelGap) / (2.0 * self.wheelRadius))
        leftSpeed = min(self.motorMaxSpeed, max(-self.motorMaxSpeed, leftSpeed))
        rightSpeed = min(self.motorMaxSpeed, max(-self.motorMaxSpeed, rightSpeed))

        self.leftMotor.setVelocity(leftSpeed)
        self.rightMotor.setVelocity(rightSpeed)

    def euler_to_quaternion(self, yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

    def odom_callback(self):
        self.publish_odom()

    def publish_odom(self):
        time.sleep(0.03)
        dt = self.time_step
        delta_x = (self.vx * math.cos(self.th) * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = self.euler_to_quaternion(self.th, 0.0, 0.0)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # set the position
        odom.pose.pose.position.x= self.x
        odom.pose.pose.position.y= self.y
        odom.pose.pose.orientation.x=odom_quat[0]
        odom.pose.pose.orientation.y=odom_quat[1]
        odom.pose.pose.orientation.z=odom_quat[2]
        odom.pose.pose.orientation.w= odom_quat[3]

        self.odom_pub.publish(odom)

    def camera_callback(self):
        image = []
        self.camera.enable(self.timestep)
        self.frontDistance_sensor.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        msg_camera = Range()
        if(len(self.camera.getRecognitionObjects()) > 0):
            object = self.camera.getRecognitionObjects()[0]
            position = (abs(object.get_position()[0]))
            color = object.get_colors()
            self.get_logger().info(str(color))
            id = object.get_id()
            pos_img = object.get_position_on_image()
            msg_camera.radiation_type = id
            msg_camera.range = position            
            if((abs(position < 0.02))):
                if(self.object1 == 0.0 and self.object2 == 0.0 and self.object3 == 0.0):
                    self.object1 = self.frontDistance_sensor.getValue()
                    self.id1 = id

                if(self.object1 != 0.0 and self.object2 == 0.0 and self.object3 == 0.0 and self.id1 != id):
                    self.object2 = self.frontDistance_sensor.getValue()
                    self.id2 = id

                if(self.object1 != 0.0 and self.object2 != 0.0 and self.object3 == 0.0 and self.id2 != id):
                    self.object3 = self.frontDistance_sensor.getValue()
                    self.id3 = id
                
                if(self.object1 > 0.0 and self.object2 > 0.0 and self.object3 > 0.0 and self.count == 0):
                    msg_camera.field_of_view = self.object1
                    msg_camera.min_range = self.object2
                    msg_camera.max_range = self.object3
                    self.camera_publisher.publish(msg_camera)
                    self.get_logger().info(str(msg_camera))
                    self.count = 1
        

    def sensor_callback(self):
        self.rightDistance_sensor.enable(self.timestep)
        msg_rightD = Range()
        msg_rightD.range = self.rightDistance_sensor.getValue()
        self.sensorPublisher_rightDistance.publish(msg_rightD)

        msg_leftD = Range()
        self.leftDistance_sensor.enable(self.timestep)
        msg_leftD.range = self.leftDistance_sensor.getValue()
        self.sensorPublisher_leftDistance.publish(msg_leftD)

        msg_frontD = Range()
        self.frontDistance_sensor.enable(self.timestep)
        msg_frontD.range = self.frontDistance_sensor.getValue()
        self.sensorPublisher_frontDistance.publish(msg_frontD)
        
        msg_rightP = Float64()
        self.rightPosition_sensor.enable(self.timestep)
        msg_rightP.data = self.rightPosition_sensor.getValue()
        self.sensorPublisher_rightPosition.publish(msg_rightP)

        msg_leftP = Float64()
        self.leftPosition_sensor.enable(self.timestep)
        msg_leftP.data = self.leftPosition_sensor.getValue()
        self.sensorPublisher_leftPosition.publish(msg_leftP)
        

def main(args=None):
    rclpy.init(args=args)
    client_vel = service_node_vel(args=args)
    rclpy.spin(client_vel)
    client_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                


            

