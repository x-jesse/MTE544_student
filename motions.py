import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# Adds required imports for handling ROS2 messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # creates velocity publisher 
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # creates QoS (Quality of Service) profile by following tut
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # IMU subscription
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos)
        self.imu_initialized = True
                
        # ENCODER subscription
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)
        self.odom_initialized = True
        
        # LaserScan subscription 
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile=qos)
        self.laser_initialized = True
        
        self.create_timer(0.1, self.timer_callback)


    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu) -> None:
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        imu_acc_x = imu_msg.linear_acceleration.x
        imu_acc_y = imu_msg.linear_acceleration.y
        imu_ang_z = imu_msg.angular_velocity.z

        self.imu_logger.log_values([imu_acc_x, imu_acc_y, imu_ang_z, timestamp])

    def odom_callback(self, odom_msg: Odometry) -> None:
        # Get timestamp from message (timestamp is always in the message header)
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        # Get message data
        odom_orientation = odom_msg.pose.pose.orientation
        odom_x_pos = odom_msg.pose.pose.position.x
        odom_y_pos = odom_msg.pose.pose.position.y

        self.odom_logger.log_values([odom_x_pos, odom_y_pos, odom_orientation, timestamp])
                
    def laser_callback(self, laser_msg: LaserScan) -> None:
        laser_ranges = laser_msg.ranges
        laser_ang_incre = laser_msg.angle_increment
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds

        self.laser_logger.log_values([laser_ranges, laser_ang_incre, timestamp])        
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    def make_circular_twist(self):
        msg=Twist()

        msg.linear.x = 0.5
        msg.angular.z = 0.7
        return msg

    def make_spiral_twist(self):
        msg=Twist()

        msg.linear.x=0.3
        msg.linear.z=0.4
        msg.angular.z=0.7
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()

        msg.linear.x=0.7
        return msg
    
    def reset_vel(self):
        msg = Twist()

        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.z = 0
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")

    rclpy.init()

    args = argParser.parse_args()
    print(f"Executing motion: {args.motion}")

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")

    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
